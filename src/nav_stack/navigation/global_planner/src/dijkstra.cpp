/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, 2013, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Eitan Marder-Eppstein
 *         David V. Lu!!
 *********************************************************************/
#include<global_planner/dijkstra.h>
#include <algorithm>
#include <queue>
#include <cmath>

namespace global_planner {

DijkstraExpansion::DijkstraExpansion(PotentialCalculator* p_calc, int nx, int ny) :
        Expander(p_calc, nx, ny), pending_(NULL), precise_(false) {
    // priority buffers
    buffer1_ = new int[PRIORITYBUFSIZE];
    buffer2_ = new int[PRIORITYBUFSIZE];
    buffer3_ = new int[PRIORITYBUFSIZE];

    priorityIncrement_ = 2 * neutral_cost_;
    
    // 初始化懒狗策略参数
    lazy_mode_ = true;
    stability_threshold_ = 0.3f;
}

DijkstraExpansion::~DijkstraExpansion() {
  delete[] buffer1_;
  delete[] buffer2_;
  delete[] buffer3_;
  if (pending_)
      delete[] pending_;
}

//
// Set/Reset map size
//
void DijkstraExpansion::setSize(int xs, int ys) {
    Expander::setSize(xs, ys);
    if (pending_)
        delete[] pending_;

    pending_ = new bool[ns_];
    memset(pending_, 0, ns_ * sizeof(bool));
}

//
// 路径平滑函数 - 使用简单的样条插值
//
void DijkstraExpansion::smoothPath(std::vector<std::pair<float, float>>& path, unsigned char* costs) {
    if (path.size() < 3) return;
    
    std::vector<std::pair<float, float>> smoothed_path;
    smoothed_path.push_back(path[0]);
    
    for (size_t i = 1; i < path.size() - 1; i++) {
        // 简单的平均平滑
        float x = (path[i-1].first + path[i].first + path[i+1].first) / 3.0f;
        float y = (path[i-1].second + path[i].second + path[i+1].second) / 3.0f;
        
        // 检查平滑后的点是否在可通行区域
        int index = toIndex(x, y);
        if (index >= 0 && index < ns_ && getCost(costs, index) < lethal_cost_) {
            smoothed_path.push_back(std::make_pair(x, y));
        } else {
            smoothed_path.push_back(path[i]);
        }
    }
    
    smoothed_path.push_back(path.back());
    path = smoothed_path;
}

//
// 检查两点之间直线是否畅通
//
bool DijkstraExpansion::isStraightLineClear(int x1, int y1, int x2, int y2, unsigned char* costs) {
    // Bresenham直线算法检查路径上的所有点
    int dx = abs(x2 - x1);
    int dy = abs(y2 - y1);
    int sx = (x1 < x2) ? 1 : -1;
    int sy = (y1 < y2) ? 1 : -1;
    int err = dx - dy;
    
    while (true) {
        // 检查当前点是否可通行
        int index = toIndex(x1, y1);
        if (index < 0 || index >= ns_ || getCost(costs, index) >= lethal_cost_) {
            return false;
        }
        
        if (x1 == x2 && y1 == y2) break;
        
        int e2 = 2 * err;
        if (e2 > -dy) {
            err -= dy;
            x1 += sx;
        }
        if (e2 < dx) {
            err += dx;
            y1 += sy;
        }
    }
    
    return true;
}

//
// 路径简化 - 去除不必要的中间点
//
void DijkstraExpansion::simplifyPath(std::vector<std::pair<float, float>>& path, unsigned char* costs) {
    if (path.size() < 3) return;
    
    std::vector<std::pair<float, float>> simplified_path;
    simplified_path.push_back(path[0]);
    
    size_t i = 0;
    while (i < path.size() - 1) {
        size_t j = path.size() - 1;
        // 从最远点开始找，看是否能直线连接
        while (j > i + 1) {
            int x1 = static_cast<int>(path[i].first);
            int y1 = static_cast<int>(path[i].second);
            int x2 = static_cast<int>(path[j].first);
            int y2 = static_cast<int>(path[j].second);
            
            if (isStraightLineClear(x1, y1, x2, y2, costs)) {
                simplified_path.push_back(path[j]);
                i = j;
                break;
            }
            j--;
        }
        
        if (j == i + 1) {
            simplified_path.push_back(path[i + 1]);
            i++;
        }
    }
    
    path = simplified_path;
}

//
// 懒狗策略：检查是否需要重新规划
//
bool DijkstraExpansion::shouldReplan(const std::vector<std::pair<float, float>>& old_path, 
                                   unsigned char* new_costs, float threshold) {
    if (!lazy_mode_ || old_path.empty()) return true;
    
    float stability = calculatePathStability(old_path, new_costs);
    return stability < threshold;
}

//
// 计算路径稳定性
//
float DijkstraExpansion::calculatePathStability(const std::vector<std::pair<float, float>>& path, 
                                              unsigned char* costs) {
    if (path.empty()) return 0.0f;
    
    int safe_cells = 0;
    int total_cells = 0;
    
    // 检查路径周围区域的稳定性
    for (const auto& point : path) {
        int x = static_cast<int>(point.first);
        int y = static_cast<int>(point.second);
        
        // 检查3x3邻域
        for (int dx = -1; dx <= 1; dx++) {
            for (int dy = -1; dy <= 1; dy++) {
                int nx = x + dx;
                int ny = y + dy;
                
                if (nx >= 0 && nx < nx_ && ny >= 0 && ny < ny_) {
                    int index = toIndex(nx, ny);
                    total_cells++;
                    if (getCost(costs, index) < lethal_cost_ * 0.7f) { // 70%的障碍物阈值
                        safe_cells++;
                    }
                }
            }
        }
    }
    
    return total_cells > 0 ? static_cast<float>(safe_cells) / total_cells : 0.0f;
}

//
// main propagation function
// Dijkstra method, breadth-first
// runs for a specified number of cycles,
//   or until it runs out of cells to update,
//   or until the Start cell is found (atStart = true)
//

bool DijkstraExpansion::calculatePotentials(unsigned char* costs, double start_x, double start_y, double end_x, double end_y,
                                           int cycles, float* potential) {
    cells_visited_ = 0;
    // priority buffers
    threshold_ = lethal_cost_;
    currentBuffer_ = buffer1_;
    currentEnd_ = 0;
    nextBuffer_ = buffer2_;
    nextEnd_ = 0;
    overBuffer_ = buffer3_;
    overEnd_ = 0;
    memset(pending_, 0, ns_ * sizeof(bool));
    std::fill(potential, potential + ns_, POT_HIGH);

    // set goal
    int k = toIndex(start_x, start_y);

    if(precise_)
    {
        double dx = start_x - (int)start_x, dy = start_y - (int)start_y;
        dx = floorf(dx * 100 + 0.5) / 100;
        dy = floorf(dy * 100 + 0.5) / 100;
        potential[k] = neutral_cost_ * 2 * dx * dy;
        potential[k+1] = neutral_cost_ * 2 * (1-dx)*dy;
        potential[k+nx_] = neutral_cost_*2*dx*(1-dy);
        potential[k+nx_+1] = neutral_cost_*2*(1-dx)*(1-dy);//*/

        push_cur(k+2);
        push_cur(k-1);
        push_cur(k+nx_-1);
        push_cur(k+nx_+2);

        push_cur(k-nx_);
        push_cur(k-nx_+1);
        push_cur(k+nx_*2);
        push_cur(k+nx_*2+1);
    }else{
        potential[k] = 0;
        push_cur(k+1);
        push_cur(k-1);
        push_cur(k-nx_);
        push_cur(k+nx_);
    }

    int nwv = 0;            // max priority block size
    int nc = 0;            // number of cells put into priority blocks
    int cycle = 0;        // which cycle we're on

    // set up start cell
    int startCell = toIndex(end_x, end_y);

    for (; cycle < cycles; cycle++) // go for this many cycles, unless interrupted
            {
        // 
        if (currentEnd_ == 0 && nextEnd_ == 0) // priority blocks empty
            return false;

        // stats
        nc += currentEnd_;
        if (currentEnd_ > nwv)
            nwv = currentEnd_;

        // reset pending_ flags on current priority buffer
        int *pb = currentBuffer_;
        int i = currentEnd_;
        while (i-- > 0)
            pending_[*(pb++)] = false;

        // process current priority buffer
        pb = currentBuffer_;
        i = currentEnd_;
        while (i-- > 0)
            updateCell(costs, potential, *pb++);

        // swap priority blocks currentBuffer_ <=> nextBuffer_
        currentEnd_ = nextEnd_;
        nextEnd_ = 0;
        pb = currentBuffer_;        // swap buffers
        currentBuffer_ = nextBuffer_;
        nextBuffer_ = pb;

        // see if we're done with this priority level
        if (currentEnd_ == 0) {
            threshold_ += priorityIncrement_;    // increment priority threshold
            currentEnd_ = overEnd_;    // set current to overflow block
            overEnd_ = 0;
            pb = currentBuffer_;        // swap buffers
            currentBuffer_ = overBuffer_;
            overBuffer_ = pb;
        }

        // check if we've hit the Start cell
        if (potential[startCell] < POT_HIGH)
            break;
    }
    //ROS_INFO("CYCLES %d/%d ", cycle, cycles);
    if (cycle < cycles)
        return true; // finished up here
    else
        return false;
}

//
// Critical function: calculate updated potential value of a cell,
//   given its neighbors' values
// Planar-wave update calculation from two lowest neighbors in a 4-grid
// Quadratic approximation to the interpolated value
// No checking of bounds here, this function should be fast
//

#define INVSQRT2 0.707106781

inline void DijkstraExpansion::updateCell(unsigned char* costs, float* potential, int n) {
    cells_visited_++;

    // do planar wave update
    float c = getCost(costs, n);
    if (c >= lethal_cost_)    // don't propagate into obstacles
        return;

    float pot = p_calc_->calculatePotential(potential, c, n);

    // now add affected neighbors to priority blocks
    if (pot < potential[n]) {
        float le = INVSQRT2 * (float)getCost(costs, n - 1);
        float re = INVSQRT2 * (float)getCost(costs, n + 1);
        float ue = INVSQRT2 * (float)getCost(costs, n - nx_);
        float de = INVSQRT2 * (float)getCost(costs, n + nx_);
        potential[n] = pot;
        //ROS_INFO("UPDATE %d %d %d %f", n, n%nx, n/nx, potential[n]);
        if (pot < threshold_)    // low-cost buffer block
                {
            if (potential[n - 1] > pot + le)
                push_next(n-1);
            if (potential[n + 1] > pot + re)
                push_next(n+1);
            if (potential[n - nx_] > pot + ue)
                push_next(n-nx_);
            if (potential[n + nx_] > pot + de)
                push_next(n+nx_);
        } else            // overflow block
        {
            if (potential[n - 1] > pot + le)
                push_over(n-1);
            if (potential[n + 1] > pot + re)
                push_over(n+1);
            if (potential[n - nx_] > pot + ue)
                push_over(n-nx_);
            if (potential[n + nx_] > pot + de)
                push_over(n+nx_);
        }
    }
}

} //end namespace global_planner
