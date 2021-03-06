// Copyright (c) 2010, University of Southern Denmark
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
// 3. Neither the name of the University of Southern Denmark nor the names of
//    its contributors may be used to endorse or promote products derived from
//    this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE UNIVERSITY OF SOUTHERN DENMARK BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

//#ifndef POSE_FILTER_H
#define POSE_FILTER_H

#include <cmath>

#include <PoseEstimation/Detection.h>

namespace PoseEstimation {

// Declarations
Detection::Vec vertical(const Detection::Vec& detections, int axis, double rx, double ry, double rz, double atol, bool flip);
Detection::Vec aboveOrBelowTable(const Detection::Vec& detections, double px, double py, double pz, double rx, double ry, double rz, double thres);  
Detection::Vec badGoodness(const Detection::Vec& detections); 
/**
 * Apply a cascade of pose filters - right now it only checks for poses with a vertical y-axis, and also makes sure they point upwards
 * @param detections detections to filter
 * @return filtered detections
 */
Detection::Vec filter(const Detection::Vec& detections, std::vector<double> constr, double thresx) {
   // Choose which axis in the poses you want to align to a reference axis
   const int axis = 1;
   
   // Choose reference axis - this I found by clicking in bm_filtered.pcd as an axis pointing upwards from the conveyor, i.e. it approximates the upward table plane normal
   const double rx = constr[0];//-0.01395;
   const double ry = constr[1];//-0.76624;
   const double rz = constr[2];//-0.64241;

  // NOTE: the reference axis MUST be L2-normalized to 1!!!

   // Table point - together with the reference axis, this specifies a full 3D plane of the conveyor
   const double px = constr[3]; //-0.043472;
   const double py = constr[4]; //-0.018472;
   const double pz = constr[5]; // 0.856371;

   
   // Filter away poses where y-axis of object frame is not close to parallel with reference axis, and also point all accepted poses in the same direction as the reference axis
   Detection::Vec result = vertical(detections, axis, rx, ry, rz, 25.0 * M_PI / 180.0, true); //25
   
   // Filter away poses where the position of the object (i.e. the translation component) is far above or below the plane specified by [rx ry rz] and [px py pz]
  // const double thres = 0.05;//0.05; // Euclidean threshold
 
   if (thresx != -1) result = aboveOrBelowTable(result, px, py, pz, rx, ry, rz, thresx);
   result = badGoodness(result);
   pcl::console::print_value("After constrains left:  %d\n", result.size());
   return result;
}
/**
 * Remove poses with bad goodness value
 *
 * 
 */
Detection::Vec badGoodness(const Detection::Vec& detections) {
   
   Detection::Vec result;
   for(size_t i = 0; i < detections.size(); ++i) {
     Detection d = detections[i];
	if ((1 - d.penalty) > 0.75)     
		result.push_back(d);
   }
   
   return result;
}

/**
 * Remove poses which do not have an x- y- or z-axis almost (anti-)parallel with the input reference axis
 * @param detections detections to filter
 * @param axis which axis of the pose to consider (0, 1 or 2 for x-, y- or z-axis)
 * @param double rx reference axis first coordinate
 * @param double ry reference axis second coordinate
 * @param double rz reference axis third coordinate
 * @param atol angle tolerance in [0,pi], defaults to 25 deg
 * @param flip if this flag is set to true, all poses with an anti-parallel axis are also flipped to be parallel with the chosen camera axis
 */
Detection::Vec vertical(const Detection::Vec& detections, int axis, double rx, double ry, double rz, double atol = 25.0 * M_PI / 180.0, bool flip = true) {
   COVIS_ASSERT(axis == 0 || axis == 1 || axis == 2);
   COVIS_ASSERT(atol >= 0.0 && atol <= M_PI);
   
   Detection::Vec result;
   for(size_t i = 0; i < detections.size(); ++i) {
      // Get the pose axis
      const double px = detections[i].pose(0,axis);
      const double py = detections[i].pose(1,axis);
      const double pz = detections[i].pose(2,axis);
      
      // Get dot
      const double dot = rx*px + ry*py + rz*pz;
      
      const bool doFlip = dot < 0.0; // Flipping is only to be done if the object axis is anti-parallel with the reference axis

      // Check angle constraint
      bool accept = false;
      if(dot <= -1.0 || dot >= 1.0) // Perfect match
         accept = true;
      else
         accept = (acos(dot) <= atol || (flip && acos(dot) >= M_PI-atol));

      // If OK
      if(accept) {
         // Get a copy
         Detection d = detections[i];
         
         // Flipping enabled?
         if(flip && doFlip) {
            // Flipping is achieved by negating chosen axis and the next modulu 3
            d.pose(0,axis) = -d.pose(0,axis);
            d.pose(1,axis) = -d.pose(1,axis);
            d.pose(2,axis) = -d.pose(2,axis);
            d.pose(0,(axis+1)%3) = -d.pose(0,(axis+1)%3);
            d.pose(1,(axis+1)%3) = -d.pose(1,(axis+1)%3);
            d.pose(2,(axis+1)%3) = -d.pose(2,(axis+1)%3);
         }
         
         // Store in result
         result.push_back(d);
      }

   }
   
   return result;
}

Detection::Vec aboveOrBelowTable(const Detection::Vec& detections, double px, double py, double pz, double rx, double ry, double rz, double thres) {
   COVIS_ASSERT(thres >= 0.0);
   
   Detection::Vec result;
   for(size_t i = 0; i < detections.size(); ++i) {
      // Get the translation component (position) of the object pose
      const double tx = detections[i].pose(0,3)/1000;
      const double ty = detections[i].pose(1,3)/1000;
      const double tz = detections[i].pose(2,3)/1000;

      // Vector going from object to the plane point
      const double tpx = tx - px;
      const double tpy = ty - py;
      const double tpz = tz - pz;
     
      // Point to plane distance: simply the dot product of the vector above with the table plane normal
      // http://mathworld.wolfram.com/Point-PlaneDistance.html
      const double p2p = fabs(tpx * rx + tpy * ry + tpz * rz);

      // Now the actual threshold: the pose must be close to the plane
      if(p2p <= thres) {
         result.push_back(detections[i]);

}
//else pcl::console::print_error("REMOVED ONE %f!\n", p2p);
   }
   
   return result;
}

} // End namespace PoseEstimation
