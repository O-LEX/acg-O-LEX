#include <filesystem>
// #include <experimental/filesystem> // uncomment here if the <filesystem> cannot be included above
//
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"
#include "Eigen/Core"
//
#include "parse_svg.h"

/***
 * signed area of a triangle connecting points (p0, p1, p2) in counter-clockwise order.
 * @param p0 1st point xy-coordinate
 * @param p1 2nd point xy-coordinate
 * @param p2 3rd point xy-coordinate
 * @return signed area (float)
 */
float area(
    const Eigen::Vector2f &p0,
    const Eigen::Vector2f &p1,
    const Eigen::Vector2f &p2) {
  const auto v01 = p1 - p0;
  const auto v02 = p2 - p0;
  // return 0.5f * (v01[0] * v02[1] - v01[1] * v02[0]); // right handed coordinate
  return 0.5f * (v01[1] * v02[0] - v01[0] * v02[1]); // left-handed coordinate (because pixel y-coordinate is going down)
}


/***
 * compute number of intersection of a ray against a line segment
 * @param org ray origin
 * @param dir ray direction (unit normal)
 * @param ps one of the two end points
 * @param pe the other end point
 * @return number of intersection
 */
int number_of_intersection_ray_against_edge(
    const Eigen::Vector2f &org,
    const Eigen::Vector2f &dir,
    const Eigen::Vector2f &ps,
    const Eigen::Vector2f &pe) {
  auto a = area(org, org + dir, ps);
  auto b = area(org, pe, org + dir);
  auto c = area(org, ps, pe);
  auto d = area(dir+ps, ps, pe);
  if (a * b > 0.f && d * c < 0.f) { return 1; }
  return 0;
  // the following code was a bug
  //auto d = area(org + dir, ps, pe);
  //if (a * b > 0.f && d * c > 0.f && fabs(d) > fabs(c)) { return 1; }
}

float calc(float a, float b, std::function<float(float)> f0, std::function<float(float)> f1, float f2, int i){
  if (i>9){return b;} //　二分法は10回行う。
  int count0 = 0, count1 = 0;
  float c = (a+b)/2;
  if(f0(a)*f1(a)<0){count0++;}
  if(f1(a)*f2<0){count0++;}
  if(f0(c)*f1(c)<0){count1++;}
  if(f1(c)*f2<0){count1++;}
  if(count0-count1==0){return calc(c, b, f0, f1, f2, i+1);}
  else{return calc(a, c, f0, f1, f2, i+1);}
}

/***
 *
 * @param org ray origin
 * @param dir ray direction (unit vector)
 * @param ps one of the two end points
 * @param pc control point
 * @param pe the other end point
 * @return the number of intersections
 */
int number_of_intersection_ray_against_quadratic_bezier(
    const Eigen::Vector2f &org,
    const Eigen::Vector2f &dir,
    const Eigen::Vector2f &ps,
    const Eigen::Vector2f &pc,
    const Eigen::Vector2f &pe) {
  // comment out below to do the assignment
  // return number_of_intersection_ray_against_edge(org, dir, ps, pe);
  // write some code below to find the intersection between ray and the quadratic
  // sturm theoryを使って二分法で解を求める
  const Eigen::Vector2f ver_dir(-dir[1], dir[0]);
  auto f0 = [=](float t){return (((1-t)*(1-t)*ps + 2*(1-t)*t*pc + t*t*pe) - org).dot(ver_dir);};
  auto f1 = [=](float t){return (2*((t-1)*ps + (1-2*t)*pc + t*pe)).dot(ver_dir);};
  float f2 = -((ps.cwiseProduct(pe)-pc.cwiseProduct(pc)).cwiseQuotient(ps-2*pc+pe)-org).dot(ver_dir);

  int count0 = 0, count1 = 0; // sturm numberを求める
  bool flag = false; //　交点が二つあるかどうか
  int ret = 0; //　返り値
  if(f0(0)*f1(0)<0){count0++;} // sturm numberの計算
  if(f1(0)*f2<0){count0++;}
  if(f0(1)*f1(1)<0){count1++;}
  if(f1(1)*f2<0){count1++;}
  if(count0-count1==0){return 0;}
  else if((count0-count1)==2){flag=true;} // 交点が二つある
  float t = calc(0, 1, f0, f1, f2, 0); // 二分法のために再帰関数calcを使った。実装は上にある
  if((((1-t)*(1-t)*ps + 2*(1-t)*t*pc + t*t*pe) - org).dot(dir)>0){ret++;} // 二つのベクトルが同じ向きかどうかを調べる

  if(flag){ // 交点が二つある場合
    count0 = 0;
    if(f0(t)*f1(t)<0){count0++;}
    if(f1(t)*f2<0){count0++;}
    /*
    二つの交点におけるtの値が非常に近い場合、両方の交点を求めることが難しい
    二分法を用いて交点を求めるcalc関数では交点の存在する範囲の上界を返す
    この上界と1の間の交点の数が0であれば、二つの交点におけるtの値は非常に近い
    tの値が非常に近い場合、二つの交点の座標も近いことが予想される
    この時、intersectionは0もしくは2になる
    よって今回は0を返すようにしている
    */
    if((count0-count1)==0){return 0;}
    t = calc(t, 1, f0, f1, f2, 0); //　二つめのtを求める
    if((((1-t)*(1-t)*ps + 2*(1-t)*t*pc + t*t*pe) - org).dot(dir)>0){ret++;}
  }
  return ret;
}

int main() {
  const auto input_file_path = std::filesystem::path(PROJECT_SOURCE_DIR) / ".." / "asset" / "r.svg";
  const auto [width, height, shape] = acg::svg_get_image_size_and_shape(input_file_path);
  if (width == 0) { // something went wrong in loading the function
    std::cout << "file open failure" << std::endl;
    abort();
  }
  const std::vector<std::string> outline_path = acg::svg_outline_path_from_shape(shape);
  const std::vector<std::vector<acg::Edge>> loops = acg::svg_loops_from_outline_path(outline_path);
  //
  std::vector<unsigned char> img_data(width * height, 255); // grayscale image initialized white
  for (unsigned int ih = 0; ih < height; ++ih) {
    for (unsigned int iw = 0; iw < width; ++iw) {
      const auto org = Eigen::Vector2f(iw + 0.5, ih + 0.5); // pixel center
      const auto dir = Eigen::Vector2f(60., 20.); // search direction
      int count_cross = 0;
      for (const auto &loop: loops) { // loop over loop (letter R have internal/external loops)
        for (const auto &edge: loop) { // loop over edge in the loop
          if (edge.is_bezier) { // in case the edge is a quadratic Bézier
            count_cross += number_of_intersection_ray_against_quadratic_bezier(
                org, dir,
                edge.ps, edge.pc, edge.pe);
          } else { // in case the edge is a line segment
            count_cross += number_of_intersection_ray_against_edge(
                org, dir,
                edge.ps, edge.pe);
          }
        }
      }
      if (count_cross % 2 == 1) { // Jordan's curve theory
        img_data[ih * width + iw] = 0; // paint black if it is inside
      }
    }
  }
  const auto output_file_path = std::filesystem::path(PROJECT_SOURCE_DIR) / "output.png";
  stbi_write_png(output_file_path.string().c_str(), width, height, 1, img_data.data(), width);
}
