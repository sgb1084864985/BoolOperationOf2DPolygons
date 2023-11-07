#pragma  once
#include "bool.hpp"

class Polygon2{
    // clock-wise
    // used to store the memory of points
    // unsigned int _n_edges;
    public:
    vector<Polygon> loops;
    Polygon2(const vector<vector<Point>>&point_loops){
        for(auto& points:point_loops){
            loops.emplace_back(points);
        }
    }
    Polygon2(){}
    bool in(const Point& p,bool include_edge=false)const;

    bool in_ray(const Point& p)const;

    // unsigned int numEdges()const{
    //    return  _n_edges; 
    // }
};

class PolygonFileController2{
    public:
    // const Polygon&& read(const std::string& filename);
    Polygon2 read(const std::string& filename);
    void read(const std::string& filename,Polygon2& polygon);
    // void write(const std::string& filename,const Polygon& polygon);
    void write(std::ofstream& filestream,const Polygon2& polygon2);
    void write(const std::string& filename,const Polygon2& polygons);
};

class PolygonIntersection2;
class BoolOperation2{
    PolygonIntersection2* implementation;
    unsigned int _last_err_code=0;
    std::string _last_ops_info="no info";
    public:
        BoolOperation2(const Polygon2& p1,const Polygon2& p2);
        vector<Polygon2> polygon_union();
        vector<Polygon2> polygon_intersect();
        vector<Polygon2> polygon_diff();
        vector<Polygon2> polygon_diff_inverse();
        unsigned getLastErrCode(){return _last_err_code;}
        std::string getLastOpsInfo(){return _last_ops_info;}
        ~BoolOperation2();
};