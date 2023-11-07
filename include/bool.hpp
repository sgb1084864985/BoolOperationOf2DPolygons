#pragma once
#include <cstddef>
#include <type_traits>
#include <list>
#include <cstdlib>
#include <fstream>
#include <vector>
#include <string>
#include <memory>
#include <cmath>

using std::vector;
#define TOL (1e-4)

class Point{public:double x,y;};
class Line{
    public:
    double a,b,c;
    Line(double a,double b,double c):a(a),b(b),c(c){}
    Line(const Point& p1,const Point& p2){
        a=p2.y-p1.y;
        b=p1.x-p2.x;
        c=p2.x*p1.y-p1.x*p2.y;

    }
    bool in(const Point& p,double tol=TOL)const{
        return std::abs(a*p.x+b*p.y+c)<tol;
    }
};

class Edge:public Line{

    public:
    const Point& start,&end;
    Edge(const Point& start,const Point&end):start(start),end(end),Line(start,end){}
    Point randomPoint()const;
    double signedDistanceFromStart(const Point& p,double tol=TOL)const;
    bool in(const Point& p,bool include_end_point=true,double tol=TOL)const;
};

class Polygon{
    // clock-wise
    // used to store the memory of points
    template<class T>
    Point* getNewPoint(const T&p){
        return new Point(p);
    }
    template<class T>
    Point* getNewPoint(const T*p){
        return new Point(*p);
    }
    public:
        // clock-wise
        vector<Edge> edges;
        vector<std::unique_ptr<Point>>end_points;

        bool in(const Point& p,bool include_edge=false)const;
        bool in_ray(const Point& p)const;
        vector<Point> points()const{
            vector<Point>points;
            for(size_t i=0;i<end_points.size();i++){
                points.push_back(*end_points[i].get());
            }
            return points;
        }
        size_t numEdges()const{
            return edges.size();
        }

        // constructors will copy the points
        // Polygon(const vector<const Point*>& points);
        // Polygon(const std::list<const Point*>& points);
        // Polygon(const vector<Point*>& points);
        // Polygon(const vector<Point>& points);

        template< template<typename...> class _points_it,class _point_t,class... TArgs>
        Polygon(const _points_it<_point_t,TArgs...>&points){
            for(auto& point:points){
                end_points.push_back(std::unique_ptr<Point>(getNewPoint(point)));
            }
            for(size_t i=0;i<end_points.size();i++){
                auto next_point=end_points[i].get();
                const Point* next_point_ptr;
                if(i==end_points.size()-1){
                    next_point_ptr=end_points[0].get();
                }
                else{
                    next_point_ptr=end_points[i+1].get();
                }
                edges.emplace_back(Edge{*next_point,*next_point_ptr});
            }
        }
        // Polygon copy()const{
        //     return std::move(Polygon(points()));
        // }
};

class PolygonFileController{
    public:
    // const Polygon&& read(const std::string& filename);
    Polygon read(const std::string& filename);
    void write(const std::string& filename,const Polygon& polygon);
    void write(std::ofstream& filestream,const Polygon& polygon);
    void write(const std::string& filename,const vector<Polygon>& polygons);
};

class PolygonIntersection;
class BoolOperation{
    PolygonIntersection* implementation;
    unsigned int _last_err_code=0;
    std::string _last_ops_info="no info";
    public:
        BoolOperation(const Polygon& p1,const Polygon& p2);
        vector<Polygon> polygon_union();
        vector<Polygon> polygon_intersect();
        vector<Polygon> polygon_diff();
        vector<Polygon> polygon_diff_inverse();
        unsigned getLastErrCode(){return _last_err_code;}
        std::string getLastOpsInfo(){return _last_ops_info;}
        ~BoolOperation();
};