#pragma once
#include "bool.hpp"
#include "bool2.hpp"
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
    double distance(const Point& p1,const Point& p2);
    bool tooClose(const Point& p1,const Point& p2,double tolerance=TOL);
    Point randomPointBetween(const Point& p1,const Point& p2);
    Point randomPointCloseToMiddle(const Point& p1,const Point& p2);
    // return 1 if same direction, -1 oppo direction, 0 none
    int sameDirection(const pair<Point, Point>&vector1,const pair<Point, Point>&vector2);
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

Intersection getIntersection(const Line& l1,const Line& l2,double tol=TOL);
Intersection getIntersectionOfEdges(const Edge& e1,const Edge& e2);
Intersection getIntersectionEdgeAndRay(const Edge& e,const Ray& r);

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

class IntersectionTable2{
    vector<vector<IntersectionTable>>_info_table;
    public:
        IntersectionTable2(const Polygon2& a,const Polygon2& b){
            for(auto&loop_a:a.loops){
                _info_table.emplace_back();
                auto& vector_handle=_info_table.back();
                for(auto&loop_b:b.loops){
                    vector_handle.emplace_back(loop_a.numEdges(),loop_b.numEdges());
                }
            }
        }
        IntersectionTable& getTable(int index1,int index2){return _info_table[index1][index2];}
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

void remove_redundant_points(std::list<const Point*>&points);