#include <stdio.h>
#include <stdlib.h>

#include <cstring>
#include <stdexcept>

// SDL2 image
#include <SDL2/SDL_image.h>

// tf2 Quaternion for yaw
#include <tf2/LinearMath/Quaternion.h>

#include <nav_msgs/msg/occupancy_grid.hpp>

#include "map_manager/image_loader.hpp"

#define MAP_IDX(sx, i, j) ((sx) * (j) + (i))

namespace map_manager {

void loadMapFromFile(nav_msgs::msg::OccupancyGrid* map,
                     const char* fname,
                     double res,
                     bool negate,
                     double occ_th,
                     double free_th,
                     double* origin,
                     MapMode mode) {
    SDL_Surface* img;
    unsigned char* pixels;
    unsigned char* p;
    unsigned char value;
    int rowstride, n_channels, avg_channels;
    unsigned int i, j;
    int k;
    double occ;
    int alpha;
    int color_sum;
    double color_avg;

    if (!(img = IMG_Load(fname))) {
        std::string errmsg =
            std::string("failed to open image file \"") + std::string(fname) + std::string("\": ") + IMG_GetError();
        throw std::runtime_error(errmsg);
    }

    map->info.width = img->w;
    map->info.height = img->h;
    map->info.resolution = res;
    map->info.origin.position.x = *(origin);
    map->info.origin.position.y = *(origin + 1);
    map->info.origin.position.z = 0.0;
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, *(origin + 2));
    map->info.origin.orientation.x = q.x();
    map->info.origin.orientation.y = q.y();
    map->info.origin.orientation.z = q.z();
    map->info.origin.orientation.w = q.w();

    map->data.resize(map->info.width * map->info.height);

    rowstride = img->pitch;
    n_channels = img->format->BytesPerPixel;

    if (mode == TRINARY || !img->format->Amask)
        avg_channels = n_channels;
    else
        avg_channels = n_channels - 1;

    pixels = (unsigned char*)(img->pixels);
    for (j = 0; j < map->info.height; j++) {
        for (i = 0; i < map->info.width; i++) {
            p = pixels + j * rowstride + i * n_channels;
            color_sum = 0;
            for (k = 0; k < avg_channels; k++) color_sum += *(p + (k));
            color_avg = color_sum / (double)avg_channels;

            if (n_channels == 1)
                alpha = 1;
            else
                alpha = *(p + n_channels - 1);

            if (negate) color_avg = 255 - color_avg;

            if (mode == RAW) {
                value = color_avg;
                map->data[MAP_IDX(map->info.width, i, map->info.height - j - 1)] = value;
                continue;
            }

            occ = (255 - color_avg) / 255.0;
            if (occ > occ_th) {
                value = +100;
            } else if (occ < free_th) {
                value = 0;
            } else if (mode == TRINARY || alpha < 1.0) {
                if (color_avg >= 126 && color_avg <= 128) {
                    value = -1;
                } else {
                    double ratio = occ;
                    value = 1 + 98 * occ;
                }
            } else {
                double ratio = (occ - free_th) / (occ_th - free_th);
                value = 1 + 98 * ratio;
            }

            map->data[MAP_IDX(map->info.width, i, map->info.height - j - 1)] = value;
        }
    }

    SDL_FreeSurface(img);
}

}  // namespace map_manager
