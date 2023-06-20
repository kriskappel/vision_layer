#ifndef VISION_LAYER_HPP_
#define VISION_LAYER_HPP_

#include <ros/ros.h>
#include <costmap_2d/layer.h>
#include <costmap_2d/layered_costmap.h>
#include <costmap_2d/GenericPluginConfig.h>
#include <dynamic_reconfigure/server.h>
#include <costmap_2d/costmap_layer.h>

#include <unordered_map>

namespace vision_layer
{

    class VisionLayer : public costmap_2d::CostmapLayer
    {
        public:
            VisionLayer(): 
                last_min_x_(-std::numeric_limits<float>::max()),
                last_min_y_(-std::numeric_limits<float>::max()),
                last_max_x_(std::numeric_limits<float>::max()),
                last_max_y_(std::numeric_limits<float>::max())
              {
                costmap_=NULL;
              }
                
            virtual void onInitialize();
            virtual void updateBounds(double robot_x, double robot_y, double robot_yaw, double * min_x, double * min_y, double * max_x, double * max_y);

            virtual void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);

            virtual void reset(){return;}

            virtual void onFootprintChanged();


        private:
            void reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level);

            double last_min_x_, last_min_y_, last_max_x_, last_max_y_;

            unsigned int rx_, ry_;

            // Indicates that the entire gradient should be recalculated next time.
            bool need_recalculation_;

            dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig> *dsrv_;

            bool flag_raytrace_;

            bool rolling_window_;
    };

    class VisionCell
    {
        public:
            VisionCell(unsigned char* costmap, unsigned char value, unsigned int &n_visible_cells, unsigned int &n_invisible_cells) : 
                costmap_(costmap), value_(value), n_visible_cells_(n_visible_cells), n_invisible_cells_(n_invisible_cells)
            {
                visibility_flag_ = true;
                // n_visible_cells_ = 0;
                // n_invisible_cells_ = 0;
            }
            inline void operator()(unsigned int offset)
            {
                if(costmap_[offset]==costmap_2d::NO_INFORMATION)
                    return;
                if(costmap_[offset]==costmap_2d::LETHAL_OBSTACLE)
                {
                    // ROS_INFO("LETHAL_OBSTACLE");
                    visibility_flag_ = false;
                    return;
                }
                
                // if (mapCells.find(offset) != mapCells.end()) 
                //     return;
                
                // mapCells[offset] = true;
                
                // ROS_INFO("operator %d", costmap_[offset]);
                
                if(visibility_flag_)
                {
                    n_visible_cells_++;
                    costmap_[offset]=costmap_2d::NO_INFORMATION;
                }
                else
                {
                    n_invisible_cells_++;
                    costmap_[offset]=costmap_2d::NO_INFORMATION;
                }
                
            }
        private:
            unsigned char* costmap_;
            unsigned char value_;
            unsigned int &n_visible_cells_;
            unsigned int &n_invisible_cells_;
            bool visibility_flag_;
            // bool cells[3600];
            // std::unordered_map<unsigned int,bool> mapCells;
    };

}  // namespace vision_layer

#endif  // VISION_LAYER_HPP_