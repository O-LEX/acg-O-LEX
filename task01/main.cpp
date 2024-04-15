#include <cstdio>
#include <iostream>
#include <cassert>
#include <vector>
#include <filesystem>
//
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

/**
 * @brief compute the area of a triangle
 * @return area
 */
float area_of_a_triangle(
    float x0, float y0,
    float x1, float y1,
    float x2, float y2) {
  return ((x1 - x2) * (y0 - y2) - (x0 - x2) * (y1 - y2)) * 0.5f;
}

/**
 * @brief draw a triangle using the barycentric coordinates
 */
void draw_triangle(
    float x0, float y0,
    float x1, float y1,
    float x2, float y2,
    std::vector<unsigned char> &img_data, unsigned int width, unsigned int height,
    unsigned char brightness) {
  for (unsigned int ih = 0; ih < height; ++ih) {
    for (unsigned int iw = 0; iw < width; ++iw) {
      const auto x = (float) iw + 0.5f;
      const auto y = (float) ih + 0.5f;
      const auto a01 = area_of_a_triangle(x, y, x0, y0, x1, y1);
      const auto a12 = area_of_a_triangle(x, y, x1, y1, x2, y2);
      const auto a20 = area_of_a_triangle(x, y, x2, y2, x0, y0);
      if (a01 > 0.f && a12 > 0.f && a20 > 0.f) {
        img_data[ih * height + iw] = brightness;
      }
    }
  }
}

/**
 * @brief draw a triangle using the barycentric coordinates
 * @param polygon_xy xy coordinates of the corners of the polygon (counter clockwise order)
 * @param brightness brightness of the painted pixel
 */
void draw_polygon(
    const std::vector<float> &polygon_xy,
    std::vector<unsigned char> &img_data, unsigned int width, unsigned int height,
    unsigned int brightness) {
  for (unsigned int ih = 0; ih < height; ++ih) {
    for (unsigned int iw = 0; iw < width; ++iw) {
      const auto x = float(iw) + 0.5f; // x-coordinate of the center of the pixel
      const auto y = float(ih) + 0.5f; // y-coordinate of the center of the pixel
      const unsigned int num_vtx = polygon_xy.size() / 2;
      float winding_number = 0.0;
      for (unsigned int iedge = 0; iedge < num_vtx; ++iedge) {
        unsigned int i0_vtx = iedge;
        unsigned int i1_vtx = (iedge + 1) % num_vtx;
        // positions of the end points of the edge relative to (x,y)
        float p0x = polygon_xy[i0_vtx * 2 + 0] - x;
        float p0y = polygon_xy[i0_vtx * 2 + 1] - y;
        float p1x = polygon_xy[i1_vtx * 2 + 0] - x;
        float p1y = polygon_xy[i1_vtx * 2 + 1] - y;
        // write a few lines of code to compute winding number (hint: use atan2)
        // winding_numberは１か-1になるはず
        // 今回は-1になってるみたいなので、tempに-1をかけた
        // -1になってること気づかなくてめっちゃ、はまった
        // デバッグ実行のやり方がわからない
        float temp = -(atan2(p1y, p1x) - atan2(p0y, p0x)) / (4 * atan2(1.0, 0.0));
        if (temp > 0.5) temp--; else if (temp < -0.5) temp++;
        winding_number += temp;
      }
      const int int_winding_number = int(std::round(winding_number));
      if (int_winding_number == 1) { // if (x,y) is inside the polygon
        img_data[ih*width + iw] = brightness;
      }
    }
  }
}

/**
 * @brief draw a line using DDA algorithm
 * @param x0 x-coordinate of the first end point
 * @param y0 y-coordinate of the first end point
 * @param x1 x-coordinate of the second end point
 * @param y1 y-coordinate of the second end point
 * @param brightness brightness of the painted pixel
 */
void dda_line(
    float x0, float y0,
    float x1, float y1,
    std::vector<unsigned char> &img_data,
    unsigned int width,
    unsigned char brightness ) {
  auto dx = x1 - x0;
  auto dy = y1 - y0;
  // write some code below to paint pixel on the line with color `brightness`
  // とりあえず端っこ塗る。二度塗りになっても気にしない。
  img_data[(int)y1 * width + (int)x1] = brightness;
  // dx==0の場合は特別扱い
  if (dx == 0) {
    int step;
    if (dy > 0) step = 1; else step = -1;
    for (int i = 0; i < abs(dy); i++) {
      img_data[(int)y0 * width + (int)x0] = brightness;
      y0 += step;
    }
    return;
  }
  auto m = dy / dx;
  float stepx, stepy; // 単位ステップで進む量
  int length; // 軸方向の線の長さ。軸は角度によって変わる
  // 角度によって処理を変える
  if (m >= 1 || m <= -1) {
    stepx = dx / dy;
    stepy = 1;
    length = abs(dy);
  } else {
    stepx = 1;
    stepy = m;
    length = abs(dx);
  }
  // 進む向き
  if (dx < 0) {
    stepx = -stepx;
    stepy = -stepy;
  }
  float px = x0;
  float py = y0;
  // メインループ
  for (int i = 0; i <= length; i++) {
    img_data[(int)py * width + (int)px] = brightness; // これだと上下逆か？
    px = x0 + stepx * i;
    py = y0 + stepy * i;
  }
}

int main() {
  constexpr unsigned int width = 100;
  constexpr unsigned int height = 100;
  std::vector<unsigned char> img_data(width * height, 255); // white initial image
  draw_triangle(5., 5., 15., 45., 45., 15., img_data, width, height, 128);
  draw_polygon(
      {55., 5.,75., 75.,15., 55.,15., 95.,95., 95.,95., 5.}, // the pix coordinates of the corners of the polygon
      img_data, width, height, // image data, size
      200); // brightness to draw
  dda_line(50.0, 50.0, 72.8, 10.0, img_data, width, 0); // right-down
  dda_line(50.0, 50.0, 90.0, 27.2, img_data, width, 0); // right-down
  dda_line(50.0, 50.0, 72.8, 90.0, img_data, width, 0); // right-up
  dda_line(50.0, 50.0, 90.0, 72.8, img_data, width, 0); // right-up
  dda_line(50.0, 50.0, 10.0, 72.8, img_data, width, 0); // left-up
  dda_line(50.0, 50.0, 27.2, 90.0, img_data, width, 0); // left-up
  dda_line(50.0, 50.0, 10.0, 27.2, img_data, width, 0); // left-down
  dda_line(50.0, 50.0, 27.2, 10.0, img_data, width, 0); // left-down
  dda_line(50.0, 50.0, 90.0, 50.0, img_data, width, 0); // right
  dda_line(50.0, 50.0, 50.0, 90.0, img_data, width, 0); // up
  dda_line(50.0, 50.0, 10.0, 50.0, img_data, width, 0); // left
  dda_line(50.0, 50.0, 50.0, 10.0, img_data, width, 0); // down
  // save gray scale image with 1 byte depth
  stbi_write_png(
      (std::filesystem::path(PROJECT_SOURCE_DIR) / "output.png").string().c_str(),
      width, height, 1,img_data.data(), width);
}
