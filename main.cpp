#include <cstddef>
#include <cstdio>
#include <iostream>
#include <string>
#include "bool.hpp"
#include "bool2.hpp"
using namespace std;

void test2(){
    auto fileController=PolygonFileController();
    auto p1=fileController.read("./resources/Rect3.ppp");
    auto p2=fileController.read("./resources/Rect1.ppp");
    auto bool_ops=BoolOperation(p1,p2);

    fileController.write("./output/union",bool_ops.polygon_union());
    fileController.write("./intersect",bool_ops.polygon_intersect());
    fileController.write("./output/diff",bool_ops.polygon_diff());
}

void test_case(int num){
    auto io=PolygonFileController();
    string path="./test_cases/case";
    path.append(std::to_string(num));

    string path_a=path,path_b=path;
    path_a.append("/a.txt");
    path_b.append("/b.txt");

    auto p1=io.read(path_a);
    auto p2=io.read(path_b);

    auto bool_ops=BoolOperation(p1,p2);

    string save_paths[]={path,path,path,path};
    string ops[]={"/union","/intersect","/diff","/diff_inverse"};
    for(size_t i=0;i<4;i++){
        save_paths[i].append(ops[i]);
    }
    io.write(save_paths[0],bool_ops.polygon_union());
    io.write(save_paths[1],bool_ops.polygon_intersect());
    io.write(save_paths[2],bool_ops.polygon_diff());
    io.write(save_paths[3],bool_ops.polygon_diff_inverse());
}

void test_case2(int num){
    auto io=PolygonFileController2();
    string path="./test_cases/case";
    path.append(std::to_string(num));

    string path_a=path,path_b=path;
    path_a.append("/a.txt");
    path_b.append("/b.txt");

    auto p1=io.read(path_a);
    auto p2=io.read(path_b);

    auto bool_ops=BoolOperation2(p1,p2);

    string save_paths[]={path,path,path,path};
    string ops[]={"/unionV2","/intersectV2","/diffV2","/diff_inverseV2"};
    for(size_t i=0;i<4;i++){
        save_paths[i].append(ops[i]);
        std::remove(save_paths[i].c_str());
    }
    io.write(save_paths[0],bool_ops.polygon_union()[0]);
    io.write(save_paths[1],bool_ops.polygon_intersect()[0]);
    io.write(save_paths[2],bool_ops.polygon_diff()[0]);
    io.write(save_paths[3],bool_ops.polygon_diff_inverse()[0]);
}

int main(){
    test_case2(0);
    return 0;
}

