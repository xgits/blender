/* SPDX-License-Identifier: Apache-2.0
 * Copyright 2011-2022 Blender Foundation */

#include "scene/light_tree.h"

CCL_NAMESPACE_BEGIN

float OrientationBounds::calculate_measure() const
{
  float theta_w = fminf(M_PI_F, theta_o + theta_e);
  float cos_theta_o = cosf(theta_o);
  float sin_theta_o = sinf(theta_o);

  return M_2PI_F * (1 - cos_theta_o) +
         M_PI_2_F * (2 * theta_w * sin_theta_o - cosf(theta_o - 2 * theta_w) -
                     2 * theta_o * sin_theta_o + cos_theta_o);
}

OrientationBounds merge(const OrientationBounds& cone_a, 
						const OrientationBounds& cone_b)
{
  /* Set cone a to always have the greater theta_o. */
  const OrientationBounds *a = &cone_a;
  const OrientationBounds *b = &cone_b;
  if (cone_b.theta_o > cone_a.theta_o) {
    a = &cone_b;
    b = &cone_a;
  }

  float theta_d = safe_acosf(dot(a->axis, b->axis));
  float theta_e = fmaxf(a->theta_e, b->theta_e);

  /* Return axis and theta_o of a if it already contains b. */
  /* This should also be called when b is empty. */
  if (a->theta_o >= fminf(M_PI_F, theta_d + b->theta_o)) {
    return OrientationBounds({a->axis, a->theta_o, theta_e});
  }
  else {
    /* Compute new theta_o that contains both a and b. */
    float theta_o = (theta_d + a->theta_o + b->theta_o) / 2;

    if (theta_o > M_PI_F) {
      return OrientationBounds({a->axis, M_PI_F, theta_e});
    }

    /* TODO: test if vectors can just be averaged. */
    /* Rotate new axis to be between a and b. */
    float theta_r = theta_o - a->theta_o;
    float3 new_axis = rotate_around_axis(a->axis, cross(a->axis, b->axis), theta_r);
    new_axis = normalize(new_axis);

    return OrientationBounds({new_axis, theta_o, theta_e});
  }
}

void LightTreeBuildNode::init_leaf(
    uint offset, uint n, const BoundBox &b, const OrientationBounds &c, float e, float e_var)
{
  bbox = b;
  bcone = c;
  energy = e;
  energy_variance = e_var;
  first_prim_index = offset;
  num_lights = n;

  children[0] = children[1] = nullptr;
  is_leaf = true;
}

void LightTreeBuildNode::init_interior(LightTreeBuildNode *c0, LightTreeBuildNode *c1)
{
  bbox = merge(c0->bbox, c1->bbox);
  bcone = merge(c0->bcone, c1->bcone);
  energy = c0->energy + c1->energy;
  energy_variance = c0->energy_variance + c1->energy_variance;
  first_prim_index = 0;
  num_lights = 0;

  children[0] = c0;
  children[1] = c1;
  is_leaf = false;
}

LightTree::LightTree(const vector<LightTreePrimitive> &prims, Scene *scene, uint max_lights_in_leaf)
{
  prims_ = prims;
  scene_ = scene;
  max_lights_in_leaf_ = max_lights_in_leaf;

  vector<LightTreePrimitiveInfo> build_data(prims.size());
  for (int i = 0; i < prims.size(); i++) {
    LightTreePrimitiveInfo prim_info;
    prim_info.bbox = calculate_bbox(prims[i]);
    prim_info.bcone = calculate_bcone(prims[i]);
    prim_info.energy = calculate_energy(prims[i]);
    prim_info.centroid = prim_info.bbox.center();
    prim_info.prim_num = i;
    build_data.push_back(prim_info);
  }

  int total_nodes = 0;
  vector<LightTreePrimitive> ordered_prims;
  LightTreeBuildNode *root;
  root = recursive_build(build_data, 0, prims.size(), total_nodes, ordered_prims);
  prims_ = ordered_prims;
}

LightTreeBuildNode *LightTree::recursive_build(vector<LightTreePrimitiveInfo> &primitive_info,
                                               int start,
                                               int end,
                                               int &total_nodes,
                                               vector<LightTreePrimitive> &ordered_prims)
{
  LightTreeBuildNode *node = new LightTreeBuildNode();
  total_nodes++;
  BoundBox node_bbox = BoundBox::empty;
  OrientationBounds node_bcone;
  BoundBox centroid_bounds = BoundBox::empty;
  float energy_total = 0.0;
  float energy_squared_total = 0.0;
  int num_prims = end - start;

  for (int i = start; i < end; i++) {
    const LightTreePrimitiveInfo &prim = primitive_info.at(i);
    node_bbox.grow(prim.bbox);
    node_bcone = merge(node_bcone, prim.bcone);
    centroid_bounds.grow(prim.centroid);

    energy_total += prim.energy;
    energy_squared_total += prim.energy * prim.energy;
  }

  /* Var(X) = E[X^2] - E[X]^2 */
  float energy_variance = (energy_squared_total / num_prims) - (energy_total / num_prims) * (energy_total / num_prims);

  if (num_prims == 1) {
    int first_prim_offset = ordered_prims.size();
    for (int i = start; i < end; i++) {
      int prim_num = primitive_info[i].prim_num;
      ordered_prims.push_back(prims_[prim_num]);
    }
    node->init_leaf(start, num_prims, node_bbox, node_bcone, energy_total, energy_variance);
  }

  return node;
}

void split_saoh(const BoundBox &centroid_bbox,
                const vector<LightTreePrimitiveInfo> &primitive_info,
                int start,
                int end,
                const BoundBox &bbox,
                const OrientationBounds &bcone)
{
  const int num_buckets = 12;

  const float inv_total_measure = 1 / bcone.calculate_measure();
  const float inv_total_surface_area = 1 / bbox.area();
  const float inv_max_extent = 1 / max3(centroid_bbox.size());

  /* Check each dimension to find the minimum splitting cost. */
  float min_cost = FLT_MAX;
  for (int dim = 0; dim < 3; dim++) {
    const float inv_extent = 1 / (centroid_bbox.size()[dim]);
    
    /* Fill in buckets with primitives. */
    vector<LightTreeBucketInfo> buckets(num_buckets);
    for (int i = start; i < end; i++) {
      const LightTreePrimitiveInfo &primitive = primitive_info[i];

      /* Normalize centroid inside of bounding box. */
      int bucket_idx = num_buckets * primitive.bbox.size()[dim] * inv_extent;
      if (bucket_idx == num_buckets)
      {
        bucket_idx = num_buckets - 1;
      }

      buckets[bucket_idx].count++;
      buckets[bucket_idx].energy += primitive.energy;
      buckets[bucket_idx].bbox.grow(primitive.bbox);
      buckets[bucket_idx].bcone = merge(buckets[bucket_idx].bcone, primitive.bcone);
    }

    /* Calculate split cost and set it if it's the minimum. */
    vector<float> bucket_costs(num_buckets - 1);
    float energy_L, energy_R;
    BoundBox bbox_L, bbox_R;
    OrientationBounds bcone_L, bcone_R;
    for (int i = 0; i < num_buckets - 1; i++) {
      energy_L = 0;
      energy_R = 0;
      bbox_L = BoundBox::empty;
      bbox_R = BoundBox::empty;
      bcone_L = OrientationBounds::empty;
      bconee_R = OrientationBounds::empty;
    }
  }
  
}

float LightTree::calculate_split_cost()
{
  return 0.0;
}

int LightTree::flatten_tree()
{
  return 1;
}

CCL_NAMESPACE_END