#include "bool2.hpp"
#include <algorithm>
#include <cstddef>
#include <iostream>
#include <ostream>
#include <fstream>
#include <string>
#include <vector>
#include "bool_impl.hpp"

using std::cout;
using std::endl;
using std::ifstream;
using std::string;

Polygon2 PolygonFileController2::read(const std::string &filename){
    Polygon2 polygons_container;
    read(filename,polygons_container);
    return polygons_container;
}
bool remove_last=false;
void PolygonFileController2::read(const std::string &filename,Polygon2& polygons_container){
    vector<Polygon>& polygons=polygons_container.loops; // 存储多边形的向量
    ifstream file(filename); // 打开文件
    if (file.is_open()) { // 如果文件打开成功
        string line; // 存储每一行的内容
        vector<Point> points; // 存储当前多边形的顶点
        while (getline(file, line)) { // 读取每一行
            if (line.find("#loop")!=std::string::npos) { // 如果是#loop，表示开始或结束一个多边形
                if (!points.empty()) { // 如果当前多边形的顶点不为空
                    if(remove_last){
                        points.pop_back();
                    }
                    polygons.push_back(Polygon(points)); // 创建一个多边形并加入向量
                    points.clear(); // 清空当前多边形的顶点
                }
            } else { // 如果不是#loop，表示是一个点的坐标
                double x, y; // 存储x和y坐标
                sscanf(line.c_str(), "%lf %lf", &x, &y); // 从字符串中解析出坐标
                points.push_back({x,y}); // 创建一个点并加入当前多边形的顶点
            }
        }
        if(!points.empty()){
            if(remove_last){
                points.pop_back();
            }
            polygons.push_back(Polygon(points)); // 创建一个多边形并加入向量
            file.close(); // 关闭文件
        }
    } else { // 如果文件打开失败
        cout << "Unable to open file" << endl;
    }
}
void PolygonFileController2::write(const std::string &filename,const Polygon2& polygons){
    std::ofstream file(filename); // An output file stream to write the file
    if (file.is_open()) { // Check if the file is opened successfully
        write(file,polygons);
        file.close(); // Close the file
    }
    else { // If the file is not opened successfully, print an error message
        cout << "Error: cannot open file " << filename << endl;
    }
}
void PolygonFileController2::write(std::ofstream&file,const Polygon2& polygon2){
    PolygonFileController io;
    for(auto&polygon:polygon2.loops){
        file<<"#loops"<<endl;
        io.write(file,polygon);
    }
}
bool Polygon2::in_ray(const Point& p)const{
    const auto random_angle=(double)rand()/RAND_MAX*2*M_PI;
    auto end_point=Point{p.x+std::cos(random_angle),p.y+std::sin(random_angle)};
    const auto random_ray=Ray(p,end_point);
    int cnt=0;

    for(auto&loop:loops){
        auto& edges=loop.edges;
        for(const auto& edge:edges){
            auto intersection=getIntersectionEdgeAndRay(edge, random_ray);
            if (intersection.intersct){
                cnt++;
            }
        }
    }
    
    return cnt%2==1;
}
bool Polygon2::in(const Point& p,bool include_edge)const{
    if(include_edge){
        for(auto&loop:loops){
            auto& edges=loop.edges;
            for(auto& edge:edges){
                if(edge.in(p,true)){
                    return true;
                }
            }
        }
    }
    int judges=0;
    const int times=7;
    for(int i=0;i<times;i++){
        judges+=in_ray(p);
    }
    return judges>times/2;
}

class EndPointStatus{
    public:
    bool skip;
    CoinsideStatus coinsideStatus;
    EndPointStatus(bool skip,CoinsideStatus status):skip(skip),coinsideStatus(status){}
    EndPointStatus():skip(false),coinsideStatus(CoinsideStatus::NONE){}
};
class PolygonIntersection2{
    // vector<PointEdge> point_edges;
    PointNextPointTable point_next_point_table;
    IntersectionTable2 intersection_tables;
    vector<vector<vector<PointStatus>>> points_in_edges_a,points_in_edges_b;
    vector<vector<EndPointStatus>> es_a_start,es_a_end,es_b_start,es_b_end;
    const Polygon2* a;
    const Polygon2* b;
    private:
        Polygon2 getPolygons(){
            Polygon2 container;
            vector<Polygon>& polygons=container.loops;
            const bool delete_redundant_points=true;
            while(!point_next_point_table.empty()){
                auto point_pair=point_next_point_table.top();
                std::list<const Point*> points;
                auto start=point_pair.first;
                auto end=point_pair.second;

                point_next_point_table.pop(start);
                points.push_back(start);
                while(end!=start){
                    points.push_back(end);

                    auto next_end=point_next_point_table.getNextPoint(end);
                    point_next_point_table.pop(end);
                    if(next_end==nullptr){
                        if(points.size()==2&&point_next_point_table.empty()){
                            cout<<"only 1 line"<<endl;
                            points.clear();
                            break;
                        }
                        cout<<"next_end is null"<<endl;
                        throw BoolException(1,"next_end is null");
                    }
                    end=next_end;
                }

                if(delete_redundant_points&&points.size()>3){
                    remove_redundant_points(points);
                }
                if(!points.empty()){
                    polygons.emplace_back(Polygon(points));
                }
            }
            return container;
        }

        void getBoardersEdge(
            const vector<PointStatus>&end_points,
            const Polygon2& tested_polygon,
            bool in_or_out,bool complement_on,bool inverse,bool ignore_coincide=false){
            // @params: 
            //  in_or_out: true: in, false out; when "in", will include the border in the tesed polygon 
            //  complement_on: if true, reserve complement coinside edges
            // @results: 
            //  results will be added to points_next_points

            for(size_t i=0;i<end_points.size()-1;i++){
                const Point* p1=end_points[i].p;
                const Point* p2=end_points[i+1].p;
                auto status=end_points[i].status;

                if(PointFuncs::tooClose(*p1,*p2)){
                    continue;
                }

                if(status==CoinsideStatus::NONE){
                    auto in_flag=tested_polygon.in(PointFuncs::randomPointCloseToMiddle(*p1,*p2));
                    if(in_flag&&in_or_out || !in_flag&&!in_or_out){
                    }
                    else{
                        continue;
                    }
                }
                else{
                    bool sd=status==CoinsideStatus::SAME_DIRECTION;
                    if(ignore_coincide || sd&&complement_on || !sd&&!complement_on){
                        continue;
                    }
                }

                if(inverse){
                    point_next_point_table.setNextPoint(p2,p1);
                }
                else{
                    point_next_point_table.setNextPoint(p1,p2);
                }
            }
        }
        void getBoarders(const vector<vector<vector<PointStatus>>>& points_in_edges,
            const Polygon2& tested_polygon,bool in_or_out,bool complement_on,bool inverse,bool ignore_coinside=false){

            for(const auto& item:points_in_edges){
                getBoarders(item,tested_polygon,in_or_out,complement_on,inverse,ignore_coinside);
            }
        }
        void getBoarders(const vector<vector<PointStatus>>& points_in_edges,
            const Polygon2& tested_polygon,bool in_or_out,bool complement_on,bool inverse,bool ignore_coinside=false){
            // @params: 
            //  in_or_out: true: in, false out; when "in", will include the border in the tesed polygon 
            //  complement_on: true: reserve the edges that coinside if they are complement
            //  inverse: add the edges in the tested polygon in oppo direction
            // @results: 
            //  results will be added to points_next_points

            for(const auto& item:points_in_edges){
                getBoardersEdge(item,tested_polygon,in_or_out,complement_on,inverse,ignore_coinside);
            }
        }
        void getIntersections(const Polygon2&a,const Polygon2&b,
            IntersectionTable2& intersection_table,
            vector<vector<EndPointStatus>>& es_a_start,
            vector<vector<EndPointStatus>>& es_a_end,
            vector<vector<EndPointStatus>>& es_b_start,
            vector<vector<EndPointStatus>>& es_b_end,
            vector<vector<vector<PointStatus>>> &points_in_edges_a,vector<vector<vector<PointStatus>>> &points_in_edges_b){
                size_t num_a_polygons=a.loops.size(),num_b_polygons=b.loops.size();
                for(int i=0;i<num_a_polygons;i++){
                    for(int j=0;j<num_b_polygons;j++){
                        getIntersections(a.loops[i],b.loops[j],intersection_table.getTable(i,j),
                            es_a_start[i],
                            es_a_end[i],
                            es_b_start[j],
                            es_b_start[j],
                            points_in_edges_a[i],points_in_edges_b[j]);
                    }
                }
                for(int t=0;t<num_a_polygons;t++){
                    auto& points_in_edges_a_=points_in_edges_a[t];
                    auto& a_=a.loops[t];
                    for(size_t i=0;i<a_.numEdges();i++){
                        points_in_edges_a_[i].push_back({a_.edges[i].start,es_a_start[t][i].coinsideStatus});
                        points_in_edges_a_[i].push_back({a_.edges[i].end,CoinsideStatus::NONE});
                        std::sort(points_in_edges_a_[i].begin(),points_in_edges_a_[i].end(),[&](const PointStatus& p1,const PointStatus& p2){
                            return a_.edges[i].signedDistanceFromStart(*p1.p)<a_.edges[i].signedDistanceFromStart(*p2.p);
                        });
                    }

                }
                for(int t=0;t<num_b_polygons;t++){
                    auto& points_in_edges_b_=points_in_edges_b[t];
                    auto& b_=b.loops[t];
                    for(size_t j=0;j<b_.numEdges();j++){
                        if(!es_b_start[t][j].skip){
                            points_in_edges_b_[j].push_back({b_.edges[j].start,es_b_start[t][j].coinsideStatus});
                        }
                        if(!es_b_end[t][j].skip){
                            points_in_edges_b_[j].push_back({b_.edges[j].end,CoinsideStatus::NONE});
                        }
                        std::sort(points_in_edges_b_[j].begin(),points_in_edges_b_[j].end(),[&](const PointStatus& p1,const PointStatus& p2){
                            return b_.edges[j].signedDistanceFromStart(*p1.p)<b_.edges[j].signedDistanceFromStart(*p2.p);
                        });
                    }

                }
        }
        void getIntersections(const Polygon&a,const Polygon&b,
            IntersectionTable& intersection_table,
            vector<EndPointStatus>& es_a_start,
            vector<EndPointStatus>& es_a_end,
            vector<EndPointStatus>& es_b_start,
            vector<EndPointStatus>& es_b_end,
            vector<vector<PointStatus>> &points_in_edges_a,vector<vector<PointStatus>> &points_in_edges_b){
            // get all intersections
            for(size_t i=0;i<a.numEdges();i++){
                // a.edges[i]
                auto start_point_coinside_status=CoinsideStatus::NONE;
                bool flag_no_add_start_point_coinside=false;
                for(size_t j=0;j<b.numEdges();j++){
                    IntersectionInfo* intersection=intersection_table.getIntersection(i,j);
                    if(!intersection){
                        // auto info=IntersectionInfo();
                        // intersection=getIntersectionOfEdges(p1.edges[i],p2.edges[j]);
                        intersection_table.setIntersection(i,j,a.edges[i],b.edges[j]);
                        intersection=intersection_table.getIntersection(i,j);
                    }
                    if(intersection->isIntersected()){
                        bool tooCloseToBStart=false;
                        bool tooCloseToBEnd=false;

                        if(PointFuncs::tooClose(b.edges[j].start, intersection->point)){
                            intersection->setPointCoincideB(true,&b.edges[i].start);
                            tooCloseToBStart=true;
                        }
                        else if(PointFuncs::tooClose(b.edges[j].end, intersection->point)){
                            intersection->setPointCoincideB(false,&b.edges[i].end);
                            tooCloseToBEnd=true;
                        }

                        if(PointFuncs::tooClose(a.edges[i].start, intersection->point)){
                            intersection->setPointCoincideA(true,&a.edges[i].start);
                        }
                        else if(PointFuncs::tooClose(a.edges[i].end, intersection->point)){
                            intersection->setPointCoincideA(false,&a.edges[i].end);
                        }
                        else{
                            if(tooCloseToBStart){
                                if(flag_no_add_start_point_coinside){
                                }
                                else{
                                    points_in_edges_a[i].emplace_back(b.edges[j].start,CoinsideStatus::NONE);
                                }
                            }
                            else if(tooCloseToBEnd){
                                // points_in_edges_a[i].emplace_back(b.edges[j].end,CoinsideStatus::NONE);
                            }
                            else{
                                points_in_edges_a[i].emplace_back(intersection->point,CoinsideStatus::NONE);
                            }
                        }
                        flag_no_add_start_point_coinside=false;
                    }
                    else{
                        if(intersection->coinside()){
                            auto relation=intersection->coinsideRelation;
                            auto status=intersection->coinsideStatus;

                            // same direction
                            bool sd=status==CoinsideStatus::SAME_DIRECTION;
                            bool not_add_b_end=intersection->be_in_a && (j==b.numEdges()-1);
                            auto func=[&](const Point& point,CoinsideStatus st){
                                if(not_add_b_end&&(&point==&b.edges[j].end)){
                                    assert(points_in_edges_a[i][0].p==&point);
                                    points_in_edges_a[i][0].status=st;
                                }
                                else{
                                    points_in_edges_a[i].emplace_back(point,st);
                                }
                            };
                            switch (relation) {
                                case IntersectionInfo::ABBA:
                                    // fallback
                                    func(sd?b.edges[j].end:b.edges[j].start,CoinsideStatus::NONE);
                                case IntersectionInfo::ABAB:
                                case IntersectionInfo::ABE:
                                    func(sd?b.edges[j].start:b.edges[j].end,status);
                                break;

                                case IntersectionInfo::BABA:
                                case IntersectionInfo::EBA:
                                    func(sd?b.edges[j].end:b.edges[j].start,CoinsideStatus::NONE);
                                    start_point_coinside_status=status;
                                break;
                                case IntersectionInfo::BAAB:
                                case IntersectionInfo::EAB:

                                case IntersectionInfo::EE:
                                case IntersectionInfo::BAE:

                                    start_point_coinside_status=status;
                                break;

                                case IntersectionInfo::BEA:
                                case IntersectionInfo::AEB:
                                break;

                                default:
                                    cout<<"not implemented"<<endl;
                                    throw std::exception();
                                break;
                            }
                            if(intersection->be_in_a){
                                flag_no_add_start_point_coinside=true;
                            }
                        }
                    }
                }
                if(start_point_coinside_status!=CoinsideStatus::NONE){
                    es_a_start[i].coinsideStatus=start_point_coinside_status;
                }
                // points_in_edges_a[i].push_back({a.edges[i].start,start_point_coinside_status});
                // points_in_edges_a[i].push_back({a.edges[i].end,CoinsideStatus::NONE});
            } 

            for(size_t j=0;j<b.numEdges();j++){
                bool skip_start_point=false;
                bool skip_end_point=false;
                auto start_point_coinside_status=CoinsideStatus::NONE;
                bool flag_no_add_start_point_coinside=false;
                for(size_t i=0;i<a.numEdges();i++){
                    IntersectionInfo* intersection=intersection_table.getIntersection(i,j);
                    assert(intersection);
                    if(intersection->isIntersected()){
                        bool tooCloseToStart=intersection->isPointCoincideB_START();
                        bool tooCloseToEnd=intersection->isPointCoincideB() && !(intersection->isPointCoincideB_START());

                        if(intersection->isPointCoincideA()){
                            if(intersection->isPointCoincideA_START()){
                                if(flag_no_add_start_point_coinside){
                                }
                                else{
                                    points_in_edges_b[j].push_back({a.edges[i].start,CoinsideStatus::NONE});
                                }
                            }
                            skip_start_point|=tooCloseToStart;
                            skip_end_point|=tooCloseToEnd;
                        }
                        else{
                            if(tooCloseToStart||tooCloseToEnd){
                            }
                            else{
                                points_in_edges_b[j].push_back({intersection->point,CoinsideStatus::NONE});
                            }
                        }
                        flag_no_add_start_point_coinside=false;
                    }
                    else{
                        if(intersection->coinside()){
                            auto relation=intersection->coinsideRelation;
                            auto status=intersection->coinsideStatus;

                            bool not_add_a_end=intersection->ae_in_b && (i==a.numEdges()-1);
                            auto func=[&](const Point& point,CoinsideStatus st){
                                if(not_add_a_end&&(&point==&a.edges[i].end)){
                                    assert(points_in_edges_b[j][0].p==&point);
                                    points_in_edges_b[j][0].status=st;
                                }
                                else{
                                    points_in_edges_b[j].emplace_back(point,st);
                                }
                            };

                            // same direction
                            bool sd=status==CoinsideStatus::SAME_DIRECTION;
                            if(!sd){
                                relation=IntersectionInfo::inverse[relation];
                            }
                            switch (relation) {
                                case IntersectionInfo::EE:
                                    skip_start_point=true;
                                    skip_end_point=true;
                                break;

                                case IntersectionInfo::EAB:
                                case IntersectionInfo::EBA:
                                case IntersectionInfo::AEB:
                                    skip_start_point=true;
                                break;

                                case IntersectionInfo::BAE:
                                case IntersectionInfo::ABE:
                                case IntersectionInfo::BEA:
                                    skip_end_point=true;
                                break;

                                default:
                                break;
                            }

                            switch (relation) {
                                case IntersectionInfo::BAAB:
                                case IntersectionInfo::EE:
                                case IntersectionInfo::BAE:
                                case IntersectionInfo::EAB:
                                    func(sd?a.edges[i].start:a.edges[i].end,status);
                                    func(sd?a.edges[i].end:a.edges[i].start,CoinsideStatus::NONE);
                                break;

                                case IntersectionInfo::BABA:
                                case IntersectionInfo::EBA:
                                    func(sd?a.edges[i].start:a.edges[i].end,status);
                                break;

                                case IntersectionInfo::BEA:
                                    func(sd?a.edges[i].start:a.edges[i].end,CoinsideStatus::NONE);
                                break;
                                case IntersectionInfo::AEB:
                                    func(sd?a.edges[i].end:a.edges[i].start,CoinsideStatus::NONE);
                                break;

                                case IntersectionInfo::ABAB:
                                case IntersectionInfo::ABE:
                                    func(sd?a.edges[i].end:a.edges[i].start,CoinsideStatus::NONE);
                                    start_point_coinside_status=status;
                                break;

                                case IntersectionInfo::ABBA:
                                    start_point_coinside_status=status;
                                break;

                                default:
                                    cout<<"not implemented"<<endl;
                                    throw std::exception();
                                break;
                            }
                            if(intersection->ae_in_b){
                                flag_no_add_start_point_coinside=true;
                            }
                        }
                    }
                }
                es_b_start[j].skip|=skip_start_point;
                es_b_end[j].skip|=skip_end_point;
                if(!skip_start_point && start_point_coinside_status!=CoinsideStatus::NONE){
                    es_b_start[j].coinsideStatus=start_point_coinside_status;
                    // points_in_edges_b[j].push_back({b.edges[j].start,start_point_coinside_status});
                }
            } 
        }
        public:
        PolygonIntersection2(const Polygon2& p1,const Polygon2& p2):
            intersection_tables(p1, p2),
            a(&p1),b(&p2)
            {

            size_t num_a_polygons=p1.loops.size(),num_b_polygons=p2.loops.size();
            for(int i=0;i<num_a_polygons;i++){
                auto numEdges=p1.loops[i].numEdges();
                points_in_edges_a.emplace_back(numEdges);
                es_a_start.emplace_back(numEdges);
                es_a_end.emplace_back(numEdges);
            }
            for(int j=0;j<num_b_polygons;j++){
                auto numEdges=p2.loops[j].numEdges();
                points_in_edges_b.emplace_back(numEdges);
                es_b_start.emplace_back(numEdges);
                es_b_end.emplace_back(numEdges);
            }
            // get all intersections

            getIntersections(p1,p2,intersection_tables,
                es_a_start,
                es_a_end,
                es_b_start,
                es_b_end,
                points_in_edges_a,points_in_edges_b);
        }

        Polygon2 do_bool(OpsType ops_type,unsigned int& err_code,std::string& info){
            // bool_ops_table
            static bool bt[][6]={
                {false,false,false,false,false,false},
                {true,false,false,true,false,false},
                {false,true,false,true,true,true},
                {true,true,true,false,true,false},
            };
            static std::string names[]={"union","intersect","diff","diff_inv"};

            // flags
            auto f=bt[(int)ops_type];

            point_next_point_table.clear();
            auto& a=*(this->a);
            auto& b=*(this->b);

            err_code=0;
            info="success";
            try{
                getBoarders(points_in_edges_a, b, f[0], f[1],f[2]);
                getBoarders(points_in_edges_b, a, f[3], f[4],f[5],true);
                return std::move(getPolygons());
            }
            catch(const BoolException& e){
                cout<<names[(int)ops_type]<<": illegal shape"<<endl;
                cout<<e.what()<<endl;
                err_code=e.getCode();
                info=names[(int)ops_type];
                info.append(": illegal shape");
                return Polygon2();
            }
        }
};

BoolOperation2::BoolOperation2(const Polygon2& p1,const Polygon2& p2):implementation(new PolygonIntersection2(p1,p2)){}
BoolOperation2::~BoolOperation2(){delete implementation;}
// vector<Polygon2> BoolOperation2::polygon_union(){return {implementation->do_bool(OpsType::UNION, _last_err_code, _last_ops_info)};}
vector<Polygon2> BoolOperation2::polygon_union(){
    vector<Polygon2> results;
    results.push_back(implementation->do_bool(OpsType::UNION, _last_err_code, _last_ops_info));
    return results;
}
vector<Polygon2> BoolOperation2::polygon_intersect(){
    vector<Polygon2> results;
    results.push_back(implementation->do_bool(OpsType::INTERSECT, _last_err_code, _last_ops_info));
    return results;
}
vector<Polygon2> BoolOperation2::polygon_diff(){
    vector<Polygon2> results;
    results.push_back(implementation->do_bool(OpsType::DIFF, _last_err_code, _last_ops_info));
    return results;
}
vector<Polygon2> BoolOperation2::polygon_diff_inverse(){
    vector<Polygon2> results;
    results.push_back(implementation->do_bool(OpsType::DIFF_INV, _last_err_code, _last_ops_info));
    return results;
}
// vector<Polygon2> BoolOperation2::polygon_intersect(){return {implementation->do_bool(OpsType::INTERSECT, _last_err_code, _last_ops_info)};}
// vector<Polygon2> BoolOperation2::polygon_diff(){return {implementation->do_bool(OpsType::DIFF, _last_err_code, _last_ops_info)};}
// vector<Polygon2> BoolOperation2::polygon_diff_inverse(){return {implementation->do_bool(OpsType::DIFF_INV, _last_err_code, _last_ops_info)};}