#include "bool.hpp"
#include <list>
#include <cstddef>
#include <exception>
#include <iostream>
#include <memory>
#include <ostream>
#include <string>
#include <unordered_map>
#include <vector>
#include <cassert>
#include <algorithm>
#include <cmath>
#include <utility>
#include <iostream>
#include <fstream>

using std::pair;
using std::cout;
using std::endl;

class Intersection{
    public:
        Point point;  // valid if intersect==true
        bool intersct;// if too lines intersect with each other
        bool coinside;// if too lines coinside
};
enum class CoinsideStatus{
    SAME_DIRECTION,OPPO_DIRECTION,NONE
};
enum class OpsType{
    UNION,INTERSECT,DIFF,DIFF_INV
};

class BoolException : public std::exception {
public:
  // 构造函数，接收错误代码和错误信息
  BoolException(unsigned int code, const std::string &message)
      : code_(code), message_(message),text_("Error code: " + std::to_string(code_) +
                       "\nError message: " + message_) {
    }

    // 重写what()函数，返回错误代码和错误信息
    const char *what() const noexcept override {
        return text_.c_str();
    }
    unsigned int getCode()const{
        return code_;
    }

private:
  // 错误代码
  // 1: illegal shape
  unsigned int code_;
  // 错误信息
  std::string message_;
  std::string text_;
};

namespace PointFuncs{
    double distance(const Point& p1,const Point& p2){return std::sqrt(pow(p1.x-p2.x,2)+pow(p1.y-p2.y,2));}
    bool tooClose(const Point& p1,const Point& p2,double tolerance=TOL){return distance(p1,p2)<tolerance;}
    Point randomPointBetween(const Point& p1,const Point& p2){
        double t=(double)rand()/RAND_MAX;
        return Point{p1.x+(p2.x-p1.x)*t,p1.y+(p2.y-p1.y)*t};
    }
    Point randomPointCloseToMiddle(const Point& p1,const Point& p2){
        double t=(double)rand()/RAND_MAX*0.4+0.3;
        return Point{p1.x+(p2.x-p1.x)*t,p1.y+(p2.y-p1.y)*t};
    }

    // return 1 if same direction, -1 oppo direction, 0 none
    int sameDirection(const pair<Point, Point>&vector1,const pair<Point, Point>&vector2){
        auto x1=vector1.second.x-vector1.first.x;
        auto y1=vector1.second.y-vector1.first.y;
        auto x2=vector2.second.x-vector2.first.x;
        auto y2=vector2.second.y-vector2.first.y;
        const double tol=TOL;

        if(std::abs(x1*y2-x2*y1)>TOL){
            // no parallel
            return 0;
        }
        else if(x1*x2+y1*y2<0){
            return -1;
        }
        return 1;
    }
}
class IntersectionInfo{
    const int NONE=0;
    const int START=1;
    const int END=2;

    bool _interscted;
    bool _coinside;
    int _point_coinside_a;
    int _point_coinside_b;
    const Point* _point_coinside;

    public:

    // for the points in the edges that coinside
    // E means too points coinside
    // use edge A's direction as reference (left to right)
    enum PointRelation{
        ABBA,ABAB,BABA,BAAB,
        EBA,EAB,ABE,BAE,AEB,BEA,EE
    };

    public:
    Point point;
    PointRelation coinsideRelation;
    CoinsideStatus coinsideStatus;
    bool bs_in_a,be_in_a,as_in_b,ae_in_b;
    static PointRelation inverse[11];

    public:

        IntersectionInfo(Intersection intersect,const Edge& a,const Edge& b):
            _interscted(intersect.intersct),point(intersect.point),_coinside(intersect.coinside),
            _point_coinside_a(NONE),_point_coinside_b(NONE),_point_coinside(nullptr)
            {
                if(_coinside){
                    if(PointFuncs::sameDirection({a.start,a.end}, {b.start,b.end})==1){
                        coinsideStatus=CoinsideStatus::SAME_DIRECTION;
                    }
                    else{
                        coinsideStatus=CoinsideStatus::OPPO_DIRECTION;
                    }
                    bs_in_a=a.in(b.start);
                    be_in_a=a.in(b.end);
                    as_in_b=b.in(a.start);
                    ae_in_b=b.in(a.end);

                    if(bs_in_a&&be_in_a){
                        if(as_in_b && ae_in_b){
                            coinsideRelation=EE;
                        }
                        else if(as_in_b){
                            coinsideRelation=EBA;
                        }
                        else if(ae_in_b){
                            coinsideRelation=ABE;
                        }
                        else{
                            coinsideRelation=ABBA;
                        }
                    }
                    else if(bs_in_a && coinsideStatus==CoinsideStatus::SAME_DIRECTION
                    ||be_in_a && coinsideStatus==CoinsideStatus::OPPO_DIRECTION){
                        if(as_in_b){
                            coinsideRelation=EAB;
                        }
                        else if(PointFuncs::tooClose(a.end, coinsideStatus==CoinsideStatus::SAME_DIRECTION?b.start:b.end)){
                            coinsideRelation=AEB;
                        }
                        else{
                            coinsideRelation=ABAB;
                        }
                    }
                    else if(bs_in_a && coinsideStatus==CoinsideStatus::OPPO_DIRECTION
                    ||be_in_a && coinsideStatus==CoinsideStatus::SAME_DIRECTION){
                        if(ae_in_b){
                            coinsideRelation=BAE;
                        }
                        else if(PointFuncs::tooClose(a.start, coinsideStatus==CoinsideStatus::SAME_DIRECTION?b.end:b.start)){
                            coinsideRelation=BEA;
                        }
                        else{
                            coinsideRelation=BABA;
                        }
                    }
                    else if(as_in_b){
                        coinsideRelation=BAAB;
                    }
                }
                else{
                    coinsideStatus=CoinsideStatus::NONE;
                }
        }
        bool isIntersected()const{return _interscted;}

        bool isPointCoincideA()const{return _point_coinside_a!=NONE;}
        bool isPointCoincideB()const{return _point_coinside_b!=NONE;}
        bool isPointCoincideA_START()const{return _point_coinside_a==START;}
        bool isPointCoincideB_START()const{return _point_coinside_b==START;}

        // start_or_end: true for start
        void setPointCoincideA(bool start_or_end,const Point* point=nullptr){
            _point_coinside_a=start_or_end?START:END;
            _point_coinside=point;
        }

        // start_or_end: true for start
        void setPointCoincideB(bool start_or_end,const Point* point=nullptr){
            _point_coinside_b=start_or_end?START:END;
            _point_coinside=point;
        }
        
        bool coinside()const{return _coinside;}
        IntersectionInfo(IntersectionInfo& intersect_info)=delete;
        IntersectionInfo(const IntersectionInfo& intersect_info)=delete;
};
IntersectionInfo::PointRelation IntersectionInfo::inverse[11]={ABBA,BABA,ABAB,BAAB,ABE,BAE,EBA,EAB,BEA,AEB,EE};

class Ray:public Line{
    const Point& start;
    const Point& end;

    public:
    Ray(const Point& start,const Point&end):start(start),end(end),Line(start,end){}
    bool in(const Point& p,bool include_end_point=false,double tol=TOL)const{
        if(!Line::in(p)){
            return false;
        }
        if(p.x==start.x&&p.y==start.y){
            return include_end_point;
        }
        if(p.x==end.x&&p.y==end.y){
            return include_end_point;
        }
        if(std::abs(a)<tol){
            return (end.x-start.x)*(p.x-start.x)>0;
        }
        return (end.y-start.y)*(p.y-start.y)>0;

    }
};

Intersection getIntersection(const Line& l1,const Line& l2,double tol=TOL){
    // l1: a1*x+b1*y+c1=0
    // l2: a2*x+b2*y+c2=0
    // x=(b1*c2-b2*c1)/(a1*b2-a2*b1)
    // y=(a2*c1-a1*c2)/(a1*b2-a2*b1)
    if(std::abs(l1.a*l2.b-l2.a*l1.b)<tol){
        // if coinside
        if(std::abs(l1.a*l2.c-l2.a*l1.c)<tol){
            return {Point(),false,true};
        }
        return {Point(),false,false};
    }
    else{
        double x=(l1.b*l2.c-l2.b*l1.c)/(l1.a*l2.b-l2.a*l1.b);
        double y=(l2.a*l1.c-l1.a*l2.c)/(l1.a*l2.b-l2.a*l1.b);
        return {Point{x,y},true,false};

    }
}
Intersection getIntersectionOfEdges(const Edge& e1,const Edge& e2){
    Intersection intersection=getIntersection(e1,e2);
    if(intersection.intersct){
        if(e1.in(intersection.point)&&e2.in(intersection.point)){
            return intersection;
        }
        else{
            return {Point(),false,false};
        }
    }
    else{
        if(intersection.coinside){
            if(e1.in(e2.start)||e1.in(e2.end)||e2.in(e1.start)||e2.in(e1.end)){
            }
            else{
                return {Point(),false,false};
            }
        }
        return intersection;
    }

}
Intersection getIntersectionEdgeAndRay(const Edge& e,const Ray& r){
    Intersection intersection=getIntersection(e,r);
    if(intersection.intersct){
        if(e.in(intersection.point)&&r.in(intersection.point)){
            return intersection;
        }
        else{
            return {Point(),false,false};
        }
    }
    else{
        return intersection;
    }
}

bool Polygon::in(const Point& p,bool include_edge)const{
    if(include_edge){
        for(auto& edge:edges){
            if(edge.in(p,true)){
                return true;
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
bool Polygon::in_ray(const Point &p)const{
    const auto random_angle=(double)rand()/RAND_MAX*2*M_PI;
    auto end_point=Point{p.x+std::cos(random_angle),p.y+std::sin(random_angle)};
    const auto random_ray=Ray(p,end_point);
    int cnt=0;
    for(const auto& edge:edges){
        auto intersection=getIntersectionEdgeAndRay(edge, random_ray);
        if (intersection.intersct){
            cnt++;
        }
    }
    return cnt%2==1;
}

double Edge::signedDistanceFromStart(const Point &p,double tol)const{
    auto distance=std::sqrt(pow(p.x-start.x,2)+pow(p.y-start.y,2));
    if(std::abs(a)<tol){
        return (end.x-start.x)*(p.x-start.x)>0?distance:-distance;
    }
    return (end.y-start.y)*(p.y-start.y)>0?distance:-distance;
}
bool Edge::in(const Point& p,bool include_end_point,double tol)const{
    if(!Line::in(p,tol)){
        return false;
    }
    if(PointFuncs::tooClose(p, start,tol) || PointFuncs::tooClose(p, end,tol)){
        return include_end_point;
    }
    auto x_max=std::max(start.x,end.x);
    auto x_min=std::min(start.x,end.x);
    auto y_max=std::max(start.y,end.y);
    auto y_min=std::min(start.y,end.y);

    return (p.x>x_min && p.x<x_max)||(p.y>y_min && p.y<y_max);
}
Point Edge::randomPoint()const{
    return PointFuncs::randomPointBetween(start,end);
}

class IntersectionTable{
    vector<vector<std::unique_ptr<IntersectionInfo>>>info_table;
    public:
        IntersectionTable(int a_edge_count,int b_edge_count){
            // init table
            for(int i=0;i<a_edge_count;i++){
                info_table.emplace_back(vector<std::unique_ptr<IntersectionInfo>>(b_edge_count));
            }
        }
        IntersectionInfo* getIntersection(int index1,int index2){return info_table[index1][index2].get();}
        void setIntersection(int index1,int index2,const Edge& e1,const Edge& e2){
            auto intersection=getIntersectionOfEdges(e1, e2);
            info_table[index1][index2]=std::unique_ptr<IntersectionInfo>(new IntersectionInfo(intersection,e1,e2));
        }
};

class PointNextPointTable{
    private:
        std::unordered_map<const Point*,Point*> m_table;
    public:
        PointNextPointTable(){
            // init table
        }
        Point* getNextPoint(const Point* p){return m_table[p];}
        void setNextPoint(const Point* src,const Point*next){
            if (m_table.find(src)!=m_table.end()) {
                // cout<<"multiple next point!"<<endl;
                throw BoolException(1,"multiple next point!");
            }
            m_table[src]=const_cast<Point*>(next);
        }
        pair<const Point*,const Point*> top(){
            auto it=m_table.begin();
            return {it->first,it->second};
        }
        void pop(const Point* p){
            m_table.erase(p);
        }
        bool empty()const{
            return m_table.empty();
        }
        void clear(){
            m_table.clear();

        }
};

class PointStatus{
    public:
    const Point* p;
    CoinsideStatus status;
    PointStatus(const Point&p,const CoinsideStatus& status):p(&p),status(status){}
};

void remove_redundant_points(std::list<const Point*>&points){
    auto src=points.begin();
    while (src!=points.end()) {
        while (true) {
            auto end1=src;
            end1++; 
            if(end1==points.end()){
                end1++;
            }
            auto end2=end1;
            end2++; 
            if(end2==points.end()){
                end2++;
            }

            if(PointFuncs::sameDirection({**src,**end1}, {**src,**end2})==1){
                points.erase(end1);
            }
            else{
                break;
            }
        }
        if(points.size()<3){
            break;
        }
        src++;
    }
}

class PolygonIntersection{
    // vector<PointEdge> point_edges;
    PointNextPointTable point_next_point_table;
    IntersectionTable intersection_table;
    vector<vector<PointStatus>> points_in_edges_a,points_in_edges_b;
    const Polygon* a;
    const Polygon* b;
    private:
        vector<Polygon> getPolygons(){
            vector<Polygon> polygons;
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
            return std::move(polygons);
        }

        void getBoardersEdge(const vector<PointStatus>&end_points,const Polygon& tested_polygon,bool in_or_out,bool complement_on,bool inverse,bool ignore_coincide=false){
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
        void getBoarders(const vector<vector<PointStatus>>& points_in_edges,
            const Polygon& tested_polygon,bool in_or_out,bool complement_on,bool inverse,bool ignore_coinside=false){
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
        void getIntersections(){
            // get all intersections
            auto& a=*(this->a);
            auto& b=*(this->b);
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
                points_in_edges_a[i].push_back({a.edges[i].start,start_point_coinside_status});
                points_in_edges_a[i].push_back({a.edges[i].end,CoinsideStatus::NONE});

                std::sort(points_in_edges_a[i].begin(),points_in_edges_a[i].end(),[&](const PointStatus& p1,const PointStatus& p2){
                    return a.edges[i].signedDistanceFromStart(*p1.p)<a.edges[i].signedDistanceFromStart(*p2.p);
                });
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
                if(!skip_start_point){
                    points_in_edges_b[j].push_back({b.edges[j].start,start_point_coinside_status});
                }
                if(!skip_end_point){
                    points_in_edges_b[j].push_back({b.edges[j].end,CoinsideStatus::NONE});
                }

                std::sort(points_in_edges_b[j].begin(),points_in_edges_b[j].end(),[&](const PointStatus& p1,const PointStatus& p2){
                    return b.edges[j].signedDistanceFromStart(*p1.p)<b.edges[j].signedDistanceFromStart(*p2.p);
                });
            } 
        }
        public:
        PolygonIntersection(const Polygon& p1,const Polygon& p2):
            intersection_table(p1.numEdges(),p2.numEdges()),a(&p1),b(&p2),points_in_edges_a(p1.numEdges()),points_in_edges_b(p2.numEdges()){
            // get all intersections

            getIntersections();
        }
        // vector<Polygon> doUnion(){ point_next_point_table.clear();
        //     point_next_point_table.clear();
        //     auto& a=*(this->a);
        //     auto& b=*(this->b);
        //     try{
        //         getBoarders(points_in_edges_a, b, false, false,false);
        //         getBoarders(points_in_edges_b, a, false, false,false,true);
        //         return std::move(getPolygons());
        //     }
        //     catch(const BoolException& e){
        //         cout<<"union: illegal shape"<<endl;
        //         cout<<e.what()<<endl;
        //         return vector<Polygon>();
        //     }
        // }
        // vector<Polygon> doIntersect(){
        //     point_next_point_table.clear();
        //     auto& a=*(this->a);
        //     auto& b=*(this->b);
        //     try{
        //         getBoarders(points_in_edges_a, b, true, false,false);
        //         getBoarders(points_in_edges_b, a, true, false,false,true);
        //         return std::move(getPolygons());
        //     }
        //     catch(const BoolException& e){
        //         cout<<"intersect: illegal shape"<<endl;
        //         cout<<e.what()<<endl;
        //         return vector<Polygon>();
        //     }
        // }
        // vector<Polygon> doDiff(){
        //     point_next_point_table.clear();
        //     auto& a=*(this->a);
        //     auto& b=*(this->b);
        //     try{
        //         getBoarders(points_in_edges_a, b, false, true,false);
        //         getBoarders(points_in_edges_b, a, true, true,true,true);
        //         return std::move(getPolygons());
        //     }
        //     catch(const BoolException& e){
        //         cout<<"diff: illegal shape"<<endl;
        //         cout<<e.what()<<endl;
        //         return vector<Polygon>();
        //     }
        // }
        // vector<Polygon> doDiffInverse(){
        //     point_next_point_table.clear();
        //     auto& a=*(this->a);
        //     auto& b=*(this->b);
        //     try{
        //         getBoarders(points_in_edges_a, b, true, true,true);
        //         getBoarders(points_in_edges_b, a, false, true,false,true);
        //         return std::move(getPolygons());
        //     }
        //     catch(const BoolException& e){
        //         cout<<"diff_inv: illegal shape"<<endl;
        //         cout<<e.what()<<endl;
        //         return vector<Polygon>();
        //     }
        // }

        vector<Polygon>do_bool(OpsType ops_type,unsigned int& err_code,std::string& info){
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
                return vector<Polygon>();
            }
        }
};

Polygon PolygonFileController::read(const std::string &filename){
    vector<Point> points; // A vector to store the points
    std::ifstream file(filename); // An input file stream to read the file
    if (file.is_open()) { // Check if the file is opened successfully
        double x, y; // Variables to store the coordinates of each point
        while (file >> x >> y) { // Read two doubles from the file until the end of file
        Point p; // Create a new point
        p.x = x; // Assign the x coordinate
        p.y = y; // Assign the y coordinate
        points.push_back(p); // Add the point to the vector
        }
        file.close(); // Close the file
    }
    else { // If the file is not opened successfully, print an error message
        cout << "Error: cannot open file " << filename << endl;
    }
    return std::move(Polygon(points)); // Return the vector of points
}
void PolygonFileController::write(const std::string &filename,const Polygon& polygon){
    std::ofstream file(filename); // An output file stream to write the file
    if (file.is_open()) { // Check if the file is opened successfully
        write(file,polygon);
        file.close(); // Close the file
    }
    else { // If the file is not opened successfully, print an error message
        cout << "Error: cannot open file " << filename << endl;
    }
}
void PolygonFileController::write(std::ofstream&file,const Polygon& polygon){
    auto& points=polygon.end_points;
    for (size_t i=0; i<points.size();i++) { // Loop through the vector of points
        auto p=points[i].get();
        file << p->x << " " << p->y << "\n"; // Write the coordinates of each point to the file
    }
}
void PolygonFileController::write(const std::string& filename,const vector<Polygon>& polygons){
    for(size_t i=0;i<polygons.size();i++){
        auto f_name=filename;
        f_name.append("_"); 
        f_name.append(std::to_string(i)); 
        write(f_name,polygons[i]);
    }
}

BoolOperation::BoolOperation(const Polygon& p1,const Polygon& p2):implementation(new PolygonIntersection(p1,p2)){}
BoolOperation::~BoolOperation(){delete implementation;}
// vector<Polygon> BoolOperation::polygon_union(){return std::move(implementation->doUnion());}
// vector<Polygon> BoolOperation::polygon_intersect(){return std::move(implementation->doIntersect());}
// vector<Polygon> BoolOperation::polygon_diff(){return std::move(implementation->doDiff());}
// vector<Polygon> BoolOperation::polygon_diff_inverse(){return std::move(implementation->doDiffInverse());}
vector<Polygon> BoolOperation::polygon_union(){return implementation->do_bool(OpsType::UNION, _last_err_code, _last_ops_info);}
vector<Polygon> BoolOperation::polygon_intersect(){return implementation->do_bool(OpsType::INTERSECT, _last_err_code, _last_ops_info);}
vector<Polygon> BoolOperation::polygon_diff(){return implementation->do_bool(OpsType::DIFF, _last_err_code, _last_ops_info);}
vector<Polygon> BoolOperation::polygon_diff_inverse(){return implementation->do_bool(OpsType::DIFF_INV, _last_err_code, _last_ops_info);}