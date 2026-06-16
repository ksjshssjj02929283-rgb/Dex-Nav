#ifndef MAP_MANAGER_MAP_LOADER_HPP_
#define MAP_MANAGER_MAP_LOADER_HPP_

#include <nav_msgs/msg/occupancy_grid.hpp>

/** Map mode
 *  Default: TRINARY -
 *      value >= occ_th - Occupied (100)
 *      value <= free_th - Free (0)
 *      otherwise - Unknown
 *  SCALE -
 *      alpha < 1.0 - Unknown
 *      value >= occ_th - Occupied (100)
 *      value <= free_th - Free (0)
 *      otherwise - f( (free_th, occ_th) ) = (0, 100)
 *          (linearly map in between values to (0,100)
 *  RAW -
 *      value = value
 */
/** Read the image from file and fill out the resp object, for later
 * use when our services are requested.
 *
 * @param res The map wil be written into here
 * @param fname The image file to read from
 * @param res The resolution of the map (gets stored in resp)
 * @param negate If true, then whiter pixels are occupied, and blacker
 *               pixels are free
 * @param occ_th Threshold above which pixels are occupied
 * @param free_th Threshold below which pixels are free
 * @param origin Triple specifying 2-D pose of lower-left corner of image
 * @param mode Map mode
 * @throws std::runtime_error If the image file can't be loaded
 * */
namespace map_manager {

enum MapMode { TRINARY, SCALE, RAW };

void loadMapFromFile(nav_msgs::msg::OccupancyGrid* map,
                     const char* fname,
                     double res,
                     bool negate,
                     double occ_th,
                     double free_th,
                     double* origin,
                     MapMode mode = TRINARY);
}  // namespace map_manager

#endif  // MAP_MANAGER_MAP_LOADER_HPP_
