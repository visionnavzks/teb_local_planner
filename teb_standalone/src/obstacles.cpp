/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016,
 *  TU Dortmund - Institute of Control Theory and Systems Engineering.
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
 *   * Neither the name of the institute nor the names of its
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
 * Author: Christoph Rösmann
 *********************************************************************/

#include <teb_local_planner/obstacles.h>

#include <iostream>

namespace teb_local_planner
{

void PolygonObstacle::fixPolygonClosure()
{
  if (vertices_.size() < 2)
    return;

  if (vertices_.front().isApprox(vertices_.back()))
    vertices_.pop_back();
}

void PolygonObstacle::calcCentroid()
{
  if (vertices_.empty())
  {
    centroid_.setConstant(NAN);
    return;
  }

  if (noVertices() == 1)
  {
    centroid_ = vertices_.front();
    return;
  }

  if (noVertices() == 2)
  {
    centroid_ = 0.5 * (vertices_.front() + vertices_.back());
    return;
  }

  centroid_.setZero();

  double area = 0.0;
  for (int i = 0; i < noVertices() - 1; ++i)
  {
    area += vertices_.at(i).coeffRef(0) * vertices_.at(i + 1).coeffRef(1) - vertices_.at(i + 1).coeffRef(0) * vertices_.at(i).coeffRef(1);
  }
  area += vertices_.at(noVertices() - 1).coeffRef(0) * vertices_.at(0).coeffRef(1) - vertices_.at(0).coeffRef(0) * vertices_.at(noVertices() - 1).coeffRef(1);
  area *= 0.5;

  if (area != 0)
  {
    for (int i = 0; i < noVertices() - 1; ++i)
    {
      double aux = vertices_.at(i).coeffRef(0) * vertices_.at(i + 1).coeffRef(1) - vertices_.at(i + 1).coeffRef(0) * vertices_.at(i).coeffRef(1);
      centroid_ += (vertices_.at(i) + vertices_.at(i + 1)) * aux;
    }
    double aux = vertices_.at(noVertices() - 1).coeffRef(0) * vertices_.at(0).coeffRef(1) - vertices_.at(0).coeffRef(0) * vertices_.at(noVertices() - 1).coeffRef(1);
    centroid_ += (vertices_.at(noVertices() - 1) + vertices_.at(0)) * aux;
    centroid_ /= (6 * area);
    return;
  }

  int i_cand = 0;
  int j_cand = 0;
  double max_dist = 0;
  for (int i = 0; i < noVertices(); ++i)
  {
    for (int j = i + 1; j < noVertices(); ++j)
    {
      double dist = (vertices_[j] - vertices_[i]).norm();
      if (dist > max_dist)
      {
        max_dist = dist;
        i_cand = i;
        j_cand = j;
      }
    }
  }
  centroid_ = 0.5 * (vertices_[i_cand] + vertices_[j_cand]);
}

Eigen::Vector2d PolygonObstacle::getClosestPoint(const Eigen::Vector2d& position) const
{
  if (noVertices() == 1)
  {
    return vertices_.front();
  }

  if (noVertices() > 1)
  {
    Eigen::Vector2d new_pt = closest_point_on_line_segment_2d(position, vertices_.at(0), vertices_.at(1));

    if (noVertices() > 2)
    {
      double dist = (new_pt - position).norm();
      Eigen::Vector2d closest_pt = new_pt;

      for (int i = 1; i < noVertices() - 1; ++i)
      {
        new_pt = closest_point_on_line_segment_2d(position, vertices_.at(i), vertices_.at(i + 1));
        double new_dist = (new_pt - position).norm();
        if (new_dist < dist)
        {
          dist = new_dist;
          closest_pt = new_pt;
        }
      }

      new_pt = closest_point_on_line_segment_2d(position, vertices_.back(), vertices_.front());
      double new_dist = (new_pt - position).norm();
      if (new_dist < dist)
        return new_pt;
      return closest_pt;
    }

    return new_pt;
  }

  std::cerr << "ERROR: PolygonObstacle::getClosestPoint() cannot find any closest point. Polygon ill-defined?" << std::endl;
  return Eigen::Vector2d::Zero();
}

bool PolygonObstacle::checkLineIntersection(const Eigen::Vector2d& line_start, const Eigen::Vector2d& line_end, double min_dist) const
{
  for (int i = 0; i < noVertices() - 1; ++i)
  {
    if (check_line_segments_intersection_2d(line_start, line_end, vertices_.at(i), vertices_.at(i + 1)))
      return true;
  }
  if (noVertices() == 2)
    return false;

  return check_line_segments_intersection_2d(line_start, line_end, vertices_.back(), vertices_.front());
}

} // namespace teb_local_planner
