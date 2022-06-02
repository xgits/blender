/* SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup modifiers
 */

#include "MOD_gpencil_lineart.h"
#include "MOD_lineart.h"

#include "lineart_intern.h"

#include "BKE_global.h"
#include "BKE_gpencil_modifier.h"
#include "BKE_lib_id.h"
#include "BKE_material.h"
#include "BKE_object.h"
#include "BKE_scene.h"
#include "DEG_depsgraph_query.h"
#include "DNA_collection_types.h"
#include "DNA_gpencil_types.h"
#include "DNA_light_types.h"
#include "DNA_material_types.h"
#include "DNA_mesh_types.h"
#include "DNA_meshdata_types.h"
#include "DNA_modifier_types.h"
#include "DNA_scene_types.h"
#include "MEM_guardedalloc.h"

#include "BLI_task.h"
#include "PIL_time.h"

/* Shadow loading etc. ================== */

LineartElementLinkNode *lineart_find_matching_eln(ListBase *shadow_elns, int obindex)
{
  LISTBASE_FOREACH (LineartElementLinkNode *, eln, shadow_elns) {
    if (eln->obindex == obindex) {
      return eln;
    }
  }
  return NULL;
}

LineartEdge *lineart_find_matching_edge(LineartElementLinkNode *shadow_eln,
                                        uint64_t edge_identifier)
{
  LineartEdge *elist = (LineartEdge *)shadow_eln->pointer;
  for (int i = 0; i < shadow_eln->element_count; i++) {
    if (elist[i].from_shadow == (LineartEdge *)edge_identifier) {
      return &elist[i];
    }
  }
  return NULL;
}

void lineart_register_shadow_cuts(LineartRenderBuffer *rb,
                                  LineartEdge *e,
                                  LineartEdge *shadow_edge)
{
  LISTBASE_FOREACH (LineartEdgeSegment *, es, &shadow_edge->segments) {
    /* Convert to view space cutting points. */
    double la1 = es->at;
    double la2 = es->next ? es->next->at : 1.0f;
    la1 = la1 * e->v2->fbcoord[3] /
          (e->v1->fbcoord[3] - la1 * (e->v1->fbcoord[3] - e->v2->fbcoord[3]));
    la2 = la2 * e->v2->fbcoord[3] /
          (e->v1->fbcoord[3] - la2 * (e->v1->fbcoord[3] - e->v2->fbcoord[3]));
    unsigned char shadow_bits = (es->occlusion != 0) ? LRT_SHADOW_MASK_SHADED :
                                                       LRT_SHADOW_MASK_LIT;
    lineart_edge_cut(rb, e, la1, la2, 0, 0, shadow_bits);
  }
}

typedef struct LineartIntersectionCutData {
  LineartElementLinkNode *eln_isect_shadow;
  LineartElementLinkNode *eln_isect_original;
  LineartRenderBuffer *rb;
} LineartIntersectionCutData;

void lineart_intersection_shadow_cut_task(void *__restrict userdata,
                                          const int orig_edge_index,
                                          const TaskParallelTLS *__restrict UNUSED(tls))
{
  LineartIntersectionCutData *data = (LineartIntersectionCutData *)userdata;

  LineartElementLinkNode *eln_isect_shadow = data->eln_isect_shadow;
  LineartElementLinkNode *eln_isect_original = data->eln_isect_original;
  LineartRenderBuffer *rb = data->rb;

  LineartEdge *e = &((LineartEdge *)eln_isect_original->pointer)[orig_edge_index];
  LineartEdge *shadow_e = lineart_find_matching_edge(eln_isect_shadow, (uint64_t)e->from_shadow);
  if (shadow_e) {
    lineart_register_shadow_cuts(rb, e, shadow_e);
  }
}

void lineart_register_intersection_shadow_cuts(LineartRenderBuffer *rb, ListBase *shadow_elns)
{
  if (!shadow_elns) {
    return;
  }

  LineartIntersectionCutData data = {0};

  data.rb = rb;

  LISTBASE_FOREACH (LineartElementLinkNode *, eln, shadow_elns) {
    if (eln->flags & LRT_ELEMENT_INTERSECTION_DATA) {
      data.eln_isect_shadow = eln;
      break;
    }
  }
  LISTBASE_FOREACH (LineartElementLinkNode *, eln, &rb->line_buffer_pointers) {
    if (eln->flags & LRT_ELEMENT_INTERSECTION_DATA) {
      data.eln_isect_original = eln;
      break;
    }
  }
  if (!data.eln_isect_shadow || !data.eln_isect_original) {
    return;
  }

  TaskParallelSettings cut_settings;
  BLI_parallel_range_settings_defaults(&cut_settings);
  /* Set the minimum amount of edges a thread has to process. */
  cut_settings.min_iter_per_thread = 2000;

  BLI_task_parallel_range(0,
                          data.eln_isect_original->element_count,
                          &data,
                          lineart_intersection_shadow_cut_task,
                          &cut_settings);
}

/* Shadow computation part ================== */

static LineartShadowSegment *lineart_give_shadow_segment(LineartRenderBuffer *rb)
{
  BLI_spin_lock(&rb->lock_cuts);

  /* See if there is any already allocated memory we can reuse. */
  if (rb->wasted_shadow_cuts.first) {
    LineartShadowSegment *es = (LineartShadowSegment *)BLI_pophead(&rb->wasted_shadow_cuts);
    BLI_spin_unlock(&rb->lock_cuts);
    memset(es, 0, sizeof(LineartShadowSegment));
    return (LineartShadowSegment *)es;
  }
  BLI_spin_unlock(&rb->lock_cuts);

  /* Otherwise allocate some new memory. */
  return (LineartShadowSegment *)lineart_mem_acquire_thread(&rb->render_data_pool,
                                                            sizeof(LineartShadowSegment));
}

static void lineart_shadow_segment_slice_get(double *fbl,
                                             double *fbr,
                                             double *gl,
                                             double *gr,
                                             double at,
                                             double at_l,
                                             double at_r,
                                             double *r_fb,
                                             double *r_g)
{
  double real_at = ((at_r - at_l) == 0) ? 0 : ((at - at_l) / (at_r - at_l));
  double ga = fbl[3] * real_at / (fbr[3] * (1.0f - real_at) + fbl[3] * real_at);
  interp_v3_v3v3_db(r_fb, fbl, fbr, real_at);
  r_fb[3] = interpd(fbr[3], fbl[3], ga);
  interp_v3_v3v3_db(r_g, gl, gr, ga);
}

/* Returns true when a new cut is needed in the middle, otherwise returns false, and `*r_new_xxx`
 * are not touched. */
static bool lineart_do_closest_segment(bool is_persp,
                                       double *s1fbl,
                                       double *s1fbr,
                                       double *s2fbl,
                                       double *s2fbr,
                                       double *s1gl,
                                       double *s1gr,
                                       double *s2gl,
                                       double *s2gr,
                                       double *r_fbl,
                                       double *r_fbr,
                                       double *r_gl,
                                       double *r_gr,
                                       double *r_new_in_the_middle,
                                       double *r_new_in_the_middle_global,
                                       double *r_new_at,
                                       bool *is_side_2r,
                                       bool *use_new_ref)
{
  int side = 0;
  int zid = is_persp ? 3 : 2;
  /* Always use the closest point to the light camera. */
  if (s1fbl[zid] >= s2fbl[zid]) {
    copy_v4_v4_db(r_fbl, s2fbl);
    copy_v3_v3_db(r_gl, s2gl);
    side++;
  }
  if (s1fbr[zid] >= s2fbr[zid]) {
    copy_v4_v4_db(r_fbr, s2fbr);
    copy_v3_v3_db(r_gr, s2gr);
    *is_side_2r = true;
    side++;
  }
  if (s1fbl[zid] <= s2fbl[zid]) {
    copy_v4_v4_db(r_fbl, s1fbl);
    copy_v3_v3_db(r_gl, s1gl);
    side--;
  }
  if (s1fbr[zid] <= s2fbr[zid]) {
    copy_v4_v4_db(r_fbr, s1fbr);
    copy_v3_v3_db(r_gr, s1gr);
    *is_side_2r = false;
    side--;
  }

  /* No need to cut in the middle, because one segment completely overlaps the other. */
  if (side) {
    if (side > 0) {
      *is_side_2r = true;
      *use_new_ref = true;
    }
    else if (side < 0) {
      *is_side_2r = false;
      *use_new_ref = false;
    }
    return false;
  }

  /* Else there must be an intersection point in the middle. Use "w" value to linearly plot the
   * position and get image space "at" position. */
  double dl = s1fbl[zid] - s2fbl[zid];
  double dr = s1fbr[zid] - s2fbr[zid];
  double ga = ratiod(dl, dr, 0);
  *r_new_at = is_persp ? s2fbr[3] * ga / (s2fbl[3] * (1.0f - ga) + s2fbr[3] * ga) : ga;
  interp_v3_v3v3_db(r_new_in_the_middle, s2fbl, s2fbr, *r_new_at);
  r_new_in_the_middle[3] = interpd(s2fbr[3], s2fbl[3], ga);
  interp_v3_v3v3_db(r_new_in_the_middle_global, s1gl, s1gr, ga);
  *use_new_ref = true;

  return true;
}

static void lineart_shadow_create_container_array(LineartRenderBuffer *rb,
                                                  bool transform_edge_cuts,
                                                  bool do_light_contour)
{
#define DISCARD_NONSENSE_SEGMENTS \
  if (es->occlusion != 0 || \
      (es->next && LRT_DOUBLE_CLOSE_ENOUGH(es->at, ((LineartEdgeSegment *)es->next)->at))) { \
    LRT_ITER_ALL_LINES_NEXT; \
    continue; \
  }

  /* Count and allocate at once to save time. */
  int segment_count = 0;
  LRT_ITER_ALL_LINES_BEGIN
  {
    /* Only contour and loose edges can actually cast shadows. */
    if (!(e->flags &
          (LRT_EDGE_FLAG_CONTOUR | LRT_EDGE_FLAG_LOOSE | LRT_EDGE_FLAG_LIGHT_CONTOUR))) {
      continue;
    }
    if (do_light_contour && e->flags == LRT_EDGE_FLAG_LIGHT_CONTOUR) {
      /* Only reproject light contours that also doubles as a view contour. */
      LineartEdge *orig_e = (LineartEdge *)e->t1;
      if (!orig_e->t2) {
        e->flags |= LRT_EDGE_FLAG_CONTOUR;
      }
      else {
        double vv[3];
        double *view_vector = vv;
        double dot_1 = 0, dot_2 = 0;
        double result;

        if (rb->cam_is_persp) {
          sub_v3_v3v3_db(view_vector, orig_e->v1->gloc, rb->camera_pos);
        }
        else {
          view_vector = rb->view_vector;
        }

        dot_1 = dot_v3v3_db(view_vector, orig_e->t1->gn);
        dot_2 = dot_v3v3_db(view_vector, orig_e->t2->gn);

        if ((result = dot_1 * dot_2) <= 0 && (dot_1 + dot_2)) {
          e->flags |= LRT_EDGE_FLAG_CONTOUR;
        }
      }
      if (!(e->flags & LRT_EDGE_FLAG_CONTOUR)) {
        continue;
      }
    }
    LISTBASE_FOREACH (LineartEdgeSegment *, es, &e->segments) {
      DISCARD_NONSENSE_SEGMENTS
      segment_count++;
    }
  }
  LRT_ITER_ALL_LINES_END

  LineartShadowSegmentContainer *ssc = lineart_mem_acquire(
      &rb->render_data_pool, sizeof(LineartShadowSegmentContainer) * segment_count);
  LineartShadowSegment *ss = lineart_mem_acquire(&rb->render_data_pool,
                                                 sizeof(LineartShadowSegment) * segment_count * 2);

  rb->shadow_containers = ssc;
  rb->shadow_containers_count = segment_count;

  int i = 0;
  LRT_ITER_ALL_LINES_BEGIN
  {
    if (!(e->flags & (LRT_EDGE_FLAG_CONTOUR | LRT_EDGE_FLAG_LOOSE))) {
      continue;
    }
    LISTBASE_FOREACH (LineartEdgeSegment *, es, &e->segments) {
      DISCARD_NONSENSE_SEGMENTS

      double next_at = es->next ? ((LineartEdgeSegment *)es->next)->at : 1.0f;
      /* Get correct XYZ and W coordinates. */
      interp_v3_v3v3_db(ssc[i].fbc1, e->v1->fbcoord, e->v2->fbcoord, es->at);
      interp_v3_v3v3_db(ssc[i].fbc2, e->v1->fbcoord, e->v2->fbcoord, next_at);

      /* Global coord for light-shadow separation line (occlusion-corrected light contour). */
      double ga1 = e->v1->fbcoord[3] * es->at /
                   (es->at * e->v1->fbcoord[3] + (1 - es->at) * e->v2->fbcoord[3]);
      double ga2 = e->v1->fbcoord[3] * next_at /
                   (next_at * e->v1->fbcoord[3] + (1 - next_at) * e->v2->fbcoord[3]);
      interp_v3_v3v3_db(ssc[i].g1, e->v1->gloc, e->v2->gloc, ga1);
      interp_v3_v3v3_db(ssc[i].g2, e->v1->gloc, e->v2->gloc, ga2);

      /* Assign an absurdly big W for initial distance so when triangles show up to catch the
       * shadow, their w must certainlly be smaller than this value so the shadow catches
       * successfully. */
      ssc[i].fbc1[3] = 1e30;
      ssc[i].fbc2[3] = 1e30;
      ssc[i].fbc1[2] = 1e30;
      ssc[i].fbc2[2] = 1e30;

      /* Assign to the first segment's right and the last segment's left position */
      copy_v4_v4_db(ss[i * 2].fbc2, ssc[i].fbc1);
      copy_v4_v4_db(ss[i * 2 + 1].fbc1, ssc[i].fbc2);
      ss[i * 2].at = 0.0f;
      ss[i * 2 + 1].at = 1.0f;
      BLI_addtail(&ssc[i].shadow_segments, &ss[i * 2]);
      BLI_addtail(&ssc[i].shadow_segments, &ss[i * 2 + 1]);

      if (e->flags & LRT_EDGE_FLAG_LIGHT_CONTOUR) {
        ssc[i].e_ref = (LineartEdge *)e->t1;
        ssc[i].e_ref_light_contour = e;
        /* Restore original edge flag. */
        e->flags &= (~LRT_EDGE_FLAG_CONTOUR);
      }
      else {
        ssc[i].e_ref = e;
      }

      ssc[i].es_ref = es;

      i++;
    }
  }
  LRT_ITER_ALL_LINES_END

  if (transform_edge_cuts) {
    LRT_ITER_ALL_LINES_BEGIN
    {
      /* Transform the cutting position to global space for regular feature lines.  */
      LISTBASE_FOREACH (LineartEdgeSegment *, es, &e->segments) {
        es->at = e->v1->fbcoord[3] * es->at /
                 (es->at * e->v1->fbcoord[3] + (1 - es->at) * e->v2->fbcoord[3]);
      }
    }
    LRT_ITER_ALL_LINES_END
  }

  if (G.debug_value == 4000) {
    printf("Shadow: Added %d raw containers\n", segment_count);
  }
}

static void lineart_shadow_edge_cut(LineartRenderBuffer *rb,
                                    LineartShadowSegmentContainer *e,
                                    double start,
                                    double end,
                                    double *start_gpos,
                                    double *end_gpos,
                                    double *start_fbc,
                                    double *end_fbc,
                                    bool facing_light,
                                    int target_reference,
                                    uint8_t silhouette_group)
{
  LineartShadowSegment *es, *ies;
  LineartShadowSegment *cut_start_after = e->shadow_segments.first,
                       *cut_end_before = e->shadow_segments.last;
  LineartShadowSegment *ns = NULL, *ns2 = NULL, *sl = NULL, *sr = NULL;
  int untouched = 0;

  /* If for some reason the occlusion function may give a result that has zero length, or
   * reversed in direction, or NAN, we take care of them here. */
  if (LRT_DOUBLE_CLOSE_ENOUGH(start, end)) {
    return;
  }
  if (LRT_DOUBLE_CLOSE_ENOUGH(start, 1) || LRT_DOUBLE_CLOSE_ENOUGH(end, 0)) {
    return;
  }
  if (UNLIKELY(start != start)) {
    start = 0;
  }
  if (UNLIKELY(end != end)) {
    end = 0;
  }

  if (start > end) {
    double t = start;
    start = end;
    end = t;
  }

  /* Begin looking for starting position of the segment. */
  /* Not using a list iteration macro because of it more clear when using for loops to iterate
   * through the segments. */
  for (es = e->shadow_segments.first; es; es = es->next) {
    if (LRT_DOUBLE_CLOSE_ENOUGH(es->at, start)) {
      cut_start_after = es;
      ns = cut_start_after;
      break;
    }
    if (es->next == NULL) {
      break;
    }
    ies = es->next;
    if (ies->at > start + 1e-09 && start > es->at) {
      cut_start_after = es;
      ns = lineart_give_shadow_segment(rb);
      break;
    }
  }
  if (!cut_start_after && LRT_DOUBLE_CLOSE_ENOUGH(1, end)) {
    untouched = 1;
  }
  for (es = cut_start_after->next; es; es = es->next) {
    /* We tried to cut at existing cutting point (e.g. where the line's occluded by a triangle
     * strip). */
    if (LRT_DOUBLE_CLOSE_ENOUGH(es->at, end)) {
      cut_end_before = es;
      ns2 = cut_end_before;
      break;
    }
    /* This check is to prevent `es->at == 1.0` (where we don't need to cut because we are at
     * the end point). */
    if (!es->next && LRT_DOUBLE_CLOSE_ENOUGH(1, end)) {
      cut_end_before = es;
      ns2 = cut_end_before;
      untouched = 1;
      break;
    }
    /* When an actual cut is needed in the line. */
    if (es->at > end) {
      cut_end_before = es;
      ns2 = lineart_give_shadow_segment(rb);
      break;
    }
  }

  /* When we still can't find any existing cut in the line, we allocate new ones. */
  if (ns == NULL) {
    ns = lineart_give_shadow_segment(rb);
  }
  if (ns2 == NULL) {
    if (untouched) {
      ns2 = ns;
      cut_end_before = ns2;
    }
    else {
      ns2 = lineart_give_shadow_segment(rb);
    }
  }

  /* If we touched the cut list, we assign the new cut position based on new cut position,
   * this way we accommodate precision lost due to multiple cut inserts. */
  ns->at = start;
  if (!untouched) {
    ns2->at = end;
  }

  double r_fbl[4], r_fbr[4], r_gl[3], r_gr[3];
  double r_new_in_the_middle[4], r_new_in_the_middle_global[3], r_new_at;

  double *s1fbl, *s1fbr, *s1gl, *s1gr;
  double tg1[3], tg2[3], tfbc1[4], tfbc2[4], mg1[3], mfbc1[4], mg2[3], mfbc2[4];
  bool is_side_2r, has_middle = false, use_new_ref;
  copy_v4_v4_db(tfbc1, start_fbc);
  copy_v3_v3_db(tg1, start_gpos);

  /* Do max stuff before insert. */
  LineartShadowSegment *nes;
  for (es = cut_start_after; es != cut_end_before; es = nes) {
    nes = es->next;

    s1fbl = es->fbc2, s1fbr = nes->fbc1;
    s1gl = es->g2, s1gr = nes->g1;
    sl = es, sr = nes;
    if (es == cut_start_after) {
      lineart_shadow_segment_slice_get(
          es->fbc2, nes->fbc1, es->g2, nes->g1, ns->at, es->at, nes->at, mfbc1, mg1);
      s1fbl = mfbc1, s1gl = mg1;
      sl = ns;
      if (cut_start_after != ns) {
        BLI_insertlinkafter(&e->shadow_segments, cut_start_after, ns);
        copy_v4_v4_db(ns->fbc1, mfbc1);
        copy_v3_v3_db(ns->g1, mg1);
      }
    }
    if (nes == cut_end_before) {
      lineart_shadow_segment_slice_get(
          es->fbc2, nes->fbc1, es->g2, nes->g1, ns2->at, es->at, nes->at, mfbc2, mg2);
      s1fbr = mfbc2, s1gr = mg2;
      sr = ns2;
      if (cut_end_before != ns2) {
        BLI_insertlinkbefore(&e->shadow_segments, cut_end_before, ns2);
        copy_v4_v4_db(ns2->fbc2, mfbc2);
        copy_v3_v3_db(ns2->g2, mg2);
        /* Need to restore the flag for next segment's reference. */
        sr->flag = es->flag;
        sr->target_reference = es->target_reference;
        sr->silhouette_group = es->silhouette_group;
      }
    }

    lineart_shadow_segment_slice_get(
        start_fbc, end_fbc, start_gpos, end_gpos, sr->at, start, end, tfbc2, tg2);

    if ((has_middle = lineart_do_closest_segment(rb->cam_is_persp,
                                                 s1fbl,
                                                 s1fbr,
                                                 tfbc1,
                                                 tfbc2,
                                                 s1gl,
                                                 s1gr,
                                                 tg1,
                                                 tg2,
                                                 r_fbl,
                                                 r_fbr,
                                                 r_gl,
                                                 r_gr,
                                                 r_new_in_the_middle,
                                                 r_new_in_the_middle_global,
                                                 &r_new_at,
                                                 &is_side_2r,
                                                 &use_new_ref))) {
      LineartShadowSegment *ss_middle = lineart_give_shadow_segment(rb);
      ss_middle->at = interpf(sr->at, sl->at, r_new_at);
      ss_middle->flag = is_side_2r ?
                            (LRT_SHADOW_CASTED | (facing_light ? LRT_SHADOW_FACING_LIGHT : 0)) :
                            LRT_SHADOW_CASTED;
      ss_middle->target_reference = (is_side_2r ? (target_reference) : sl->target_reference);
      ss_middle->silhouette_group = (is_side_2r ? (silhouette_group) : sl->silhouette_group);
      copy_v3_v3_db(ss_middle->g1, r_new_in_the_middle_global);
      copy_v3_v3_db(ss_middle->g2, r_new_in_the_middle_global);
      copy_v4_v4_db(ss_middle->fbc1, r_new_in_the_middle);
      copy_v4_v4_db(ss_middle->fbc2, r_new_in_the_middle);
      BLI_insertlinkafter(&e->shadow_segments, sl, ss_middle);
    }
    /* Always assign the "closest" value to the segment. */
    copy_v4_v4_db(sl->fbc2, r_fbl);
    copy_v3_v3_db(sl->g2, r_gl);
    copy_v4_v4_db(sr->fbc1, r_fbr);
    copy_v3_v3_db(sr->g1, r_gr);

    if (has_middle) {
      sl->flag = LRT_SHADOW_CASTED |
                 (is_side_2r ? 0 : (facing_light ? LRT_SHADOW_FACING_LIGHT : 0));
      sl->target_reference = is_side_2r ? es->target_reference : target_reference;
      sl->silhouette_group = is_side_2r ? es->silhouette_group : silhouette_group;
    }
    else {
      sl->flag = LRT_SHADOW_CASTED |
                 (is_side_2r ? (facing_light ? LRT_SHADOW_FACING_LIGHT : 0) : 0);
      sl->target_reference = use_new_ref ? target_reference : es->target_reference;
      sl->silhouette_group = use_new_ref ? silhouette_group : es->silhouette_group;
    }

    copy_v4_v4_db(tfbc1, tfbc2);
    copy_v3_v3_db(tg1, tg2);
  }
}

/* Because we have already done occlusion in the shadow camera, so any visual intersection
 * found
 * here must mean that the triangle is behind the given line so it will always project a
 * shadow. */
static bool lineart_shadow_cast_onto_triangle(LineartRenderBuffer *rb,
                                              LineartTriangle *tri,
                                              LineartShadowSegmentContainer *ssc,
                                              double *r_at_l,
                                              double *r_at_r,
                                              double *r_fb_l,
                                              double *r_fb_r,
                                              double *r_global_l,
                                              double *r_global_r,
                                              bool *r_facing_light)
{

  double *LFBC = ssc->fbc1, *RFBC = ssc->fbc2, *FBC0 = tri->v[0]->fbcoord,
         *FBC1 = tri->v[1]->fbcoord, *FBC2 = tri->v[2]->fbcoord;

  /* Bound box check. */
  if ((MAX3(FBC0[0], FBC1[0], FBC2[0]) < MIN2(LFBC[0], RFBC[0])) ||
      (MIN3(FBC0[0], FBC1[0], FBC2[0]) > MAX2(LFBC[0], RFBC[0])) ||
      (MAX3(FBC0[1], FBC1[1], FBC2[1]) < MIN2(LFBC[1], RFBC[1])) ||
      (MIN3(FBC0[1], FBC1[1], FBC2[1]) > MAX2(LFBC[1], RFBC[1]))) {
    return false;
  }

  bool is_persp = rb->cam_is_persp;
  double ratio[2];
  int trie[2];
  int pi = 0;
  if (lineart_line_isec_2d_ignore_line2pos(FBC0, FBC1, LFBC, RFBC, &ratio[pi])) {
    trie[pi] = 0;
    pi++;
  }
  if (lineart_line_isec_2d_ignore_line2pos(FBC1, FBC2, LFBC, RFBC, &ratio[pi])) {
    /* ratio[0] == 1 && ratio[1] == 0 means we found a intersection at the same point of the
     * edge (FBC1), ignore this one and try get the intersection point from the other side of
     * the edge
     */
    if (!(pi && LRT_DOUBLE_CLOSE_ENOUGH(ratio[0], 1.0f) &&
          LRT_DOUBLE_CLOSE_ENOUGH(ratio[1], 0.0f))) {
      trie[pi] = 1;
      pi++;
    }
  }
  if (!pi) {
    return false;
  }
  else if (pi == 1 && lineart_line_isec_2d_ignore_line2pos(FBC2, FBC0, LFBC, RFBC, &ratio[pi])) {

    if ((trie[0] == 0 && LRT_DOUBLE_CLOSE_ENOUGH(ratio[0], 0.0f) &&
         LRT_DOUBLE_CLOSE_ENOUGH(ratio[1], 1.0f)) ||
        (trie[0] == 1 && LRT_DOUBLE_CLOSE_ENOUGH(ratio[0], 1.0f) &&
         LRT_DOUBLE_CLOSE_ENOUGH(ratio[1], 0.0f))) {
      return false;
    }
    trie[pi] = 2;
    pi++;
  }

  if (pi != 2) {
    return false;
  }

  /* Get projected global position. */

  double gpos1[3], gpos2[3];
  double *v1 = (trie[0] == 0 ? FBC0 : (trie[0] == 1 ? FBC1 : FBC2));
  double *v2 = (trie[0] == 0 ? FBC1 : (trie[0] == 1 ? FBC2 : FBC0));
  double *v3 = (trie[1] == 0 ? FBC0 : (trie[1] == 1 ? FBC1 : FBC2));
  double *v4 = (trie[1] == 0 ? FBC1 : (trie[1] == 1 ? FBC2 : FBC0));
  double *gv1 = (trie[0] == 0 ? tri->v[0]->gloc :
                                (trie[0] == 1 ? tri->v[1]->gloc : tri->v[2]->gloc));
  double *gv2 = (trie[0] == 0 ? tri->v[1]->gloc :
                                (trie[0] == 1 ? tri->v[2]->gloc : tri->v[0]->gloc));
  double *gv3 = (trie[1] == 0 ? tri->v[0]->gloc :
                                (trie[1] == 1 ? tri->v[1]->gloc : tri->v[2]->gloc));
  double *gv4 = (trie[1] == 0 ? tri->v[1]->gloc :
                                (trie[1] == 1 ? tri->v[2]->gloc : tri->v[0]->gloc));
  double gr1 = is_persp ? v1[3] * ratio[0] / (ratio[0] * v1[3] + (1 - ratio[0]) * v2[3]) :
                          ratio[0];
  double gr2 = is_persp ? v3[3] * ratio[1] / (ratio[1] * v3[3] + (1 - ratio[1]) * v4[3]) :
                          ratio[1];
  interp_v3_v3v3_db(gpos1, gv1, gv2, gr1);
  interp_v3_v3v3_db(gpos2, gv3, gv4, gr2);

  double fbc1[4], fbc2[4];

  mul_v4_m4v3_db(fbc1, rb->view_projection, gpos1);
  mul_v4_m4v3_db(fbc2, rb->view_projection, gpos2);
  if (is_persp) {
    mul_v3db_db(fbc1, 1.0f / fbc1[3]);
    mul_v3db_db(fbc2, 1.0f / fbc2[3]);
  }

  int use = (fabs(LFBC[0] - RFBC[0]) > fabs(LFBC[1] - RFBC[1])) ? 0 : 1;
  double at1 = ratiod(LFBC[use], RFBC[use], fbc1[use]);
  double at2 = ratiod(LFBC[use], RFBC[use], fbc2[use]);
  if (at1 > at2) {
    swap_v3_v3_db(gpos1, gpos2);
    swap_v4_v4_db(fbc1, fbc2);
    SWAP(double, at1, at2);
  }

  /* Not effectively projecting anything. */

  if (at1 > (1.0f - FLT_EPSILON) || at2 < FLT_EPSILON) {
    return false;
  }

  /* Trim to edge's end points. */

  double t_fbc1[4], t_fbc2[4], t_gpos1[3], t_gpos2[3];
  bool trimmed1 = false, trimmed2 = false;
  if (at1 < 0 || at2 > 1) {
    double rat1 = (-at1) / (at2 - at1);
    double rat2 = (1.0f - at1) / (at2 - at1);
    double gat1 = is_persp ? fbc1[3] * rat1 / (rat1 * fbc1[3] + (1 - rat1) * fbc2[3]) : rat1;
    double gat2 = is_persp ? fbc1[3] * rat2 / (rat2 * fbc1[3] + (1 - rat2) * fbc2[3]) : rat2;
    if (at1 < 0) {
      interp_v3_v3v3_db(t_gpos1, gpos1, gpos2, gat1);
      interp_v3_v3v3_db(t_fbc1, fbc1, fbc2, rat1);
      t_fbc1[3] = interpd(fbc2[3], fbc1[3], gat1);
      at1 = 0, trimmed1 = true;
    }
    if (at2 > 1) {
      interp_v3_v3v3_db(t_gpos2, gpos1, gpos2, gat2);
      interp_v3_v3v3_db(t_fbc2, fbc1, fbc2, rat2);
      t_fbc2[3] = interpd(fbc2[3], fbc1[3], gat2);
      at2 = 1, trimmed2 = true;
    }
  }
  if (trimmed1) {
    copy_v4_v4_db(fbc1, t_fbc1);
    copy_v3_v3_db(gpos1, t_gpos1);
  }
  if (trimmed2) {
    copy_v4_v4_db(fbc2, t_fbc2);
    copy_v3_v3_db(gpos2, t_gpos2);
  }

  *r_at_l = at1;
  *r_at_r = at2;
  copy_v4_v4_db(r_fb_l, fbc1);
  copy_v4_v4_db(r_fb_r, fbc2);
  copy_v3_v3_db(r_global_l, gpos1);
  copy_v3_v3_db(r_global_r, gpos2);

  double cv[3];

  if (is_persp) {
    sub_v3_v3v3_db(cv, rb->camera_pos, tri->v[0]->gloc);
  }
  else {
    copy_v3_v3_db(cv, rb->view_vector);
  }

  double dot_f = dot_v3v3_db(cv, tri->gn);
  *r_facing_light = (dot_f < 0);

  return true;
}

static void lineart_shadow_cast_task(void *__restrict userdata,
                                     const int ssc_index,
                                     const TaskParallelTLS *__restrict UNUSED(tls))
{
  LineartRenderBuffer *rb = (LineartRenderBuffer *)userdata;
  LineartShadowSegmentContainer *ssc = &rb->shadow_containers[ssc_index];

  LineartTriangleThread *tri;
  double at_l, at_r;
  double fb_l[4], fb_r[4];
  double global_l[3], global_r[3];
  bool facing_light;

  LRT_EDGE_BA_MARCHING_BEGIN(ssc->fbc1, ssc->fbc2)
  {
    for (int i = 0; i < nba->triangle_count; i++) {
      tri = (LineartTriangleThread *)nba->linked_triangles[i];
      if (tri->testing_e[0] == (LineartEdge *)ssc ||
          lineart_edge_from_triangle(
              (LineartTriangle *)tri, ssc->e_ref, rb->allow_overlapping_edges)) {
        continue;
      }
      tri->testing_e[0] = (LineartEdge *)ssc;

      if (lineart_shadow_cast_onto_triangle(rb,
                                            (LineartTriangle *)tri,
                                            ssc,
                                            &at_l,
                                            &at_r,
                                            fb_l,
                                            fb_r,
                                            global_l,
                                            global_r,
                                            &facing_light)) {
        lineart_shadow_edge_cut(rb,
                                ssc,
                                at_l,
                                at_r,
                                global_l,
                                global_r,
                                fb_l,
                                fb_r,
                                facing_light,
                                tri->base.target_reference,
                                tri->base.silhouette_group);
      }
    }
    LRT_EDGE_BA_MARCHING_NEXT(ssc->fbc1, ssc->fbc2);
  }
  LRT_EDGE_BA_MARCHING_END;
}

static void lineart_shadow_cast(LineartRenderBuffer *rb,
                                bool transform_edge_cuts,
                                bool do_light_contour)
{

  lineart_shadow_create_container_array(rb, transform_edge_cuts, do_light_contour);

  TaskParallelSettings cast_settings;
  BLI_parallel_range_settings_defaults(&cast_settings);
  /* Set the minimum amount of edges a thread has to process. */
  cast_settings.min_iter_per_thread = 2000;

  BLI_task_parallel_range(
      0, rb->shadow_containers_count, rb, lineart_shadow_cast_task, &cast_settings);
}

static bool lineart_shadow_cast_generate_edges(LineartRenderBuffer *rb,
                                               bool do_original_edges,
                                               LineartElementLinkNode **r_veln,
                                               LineartElementLinkNode **r_eeln)
{
  int tot_edges = 0;
  int tot_orig_edges = 0;
  for (int i = 0; i < rb->shadow_containers_count; i++) {
    LineartShadowSegmentContainer *ssc = &rb->shadow_containers[i];
    LISTBASE_FOREACH (LineartShadowSegment *, ss, &ssc->shadow_segments) {
      if (!(ss->flag & LRT_SHADOW_CASTED)) {
        continue;
      }
      if (!ss->next) {
        break;
      }
      tot_edges++;
    }
    tot_orig_edges++;
  }

  int edge_alloc = tot_edges + (do_original_edges ? tot_orig_edges : 0);

  if (G.debug_value == 4000) {
    printf("Line art shadow segments total: %d\n", tot_edges);
  }

  if (!edge_alloc) {
    return false;
  }
  LineartElementLinkNode *veln = lineart_mem_acquire(rb->shadow_data_pool,
                                                     sizeof(LineartElementLinkNode));
  LineartElementLinkNode *eeln = lineart_mem_acquire(rb->shadow_data_pool,
                                                     sizeof(LineartElementLinkNode));
  veln->pointer = lineart_mem_acquire(rb->shadow_data_pool, sizeof(LineartVert) * edge_alloc * 2);
  eeln->pointer = lineart_mem_acquire(rb->shadow_data_pool, sizeof(LineartEdge) * edge_alloc);
  LineartEdgeSegment *es = lineart_mem_acquire(rb->shadow_data_pool,
                                               sizeof(LineartEdgeSegment) * edge_alloc);
  *r_veln = veln;
  *r_eeln = eeln;

  veln->element_count = edge_alloc * 2;
  eeln->element_count = edge_alloc;

  LineartVert *vlist = veln->pointer;
  LineartEdge *elist = eeln->pointer;

  int ei = 0;
  for (int i = 0; i < rb->shadow_containers_count; i++) {
    LineartShadowSegmentContainer *ssc = &rb->shadow_containers[i];
    LISTBASE_FOREACH (LineartShadowSegment *, ss, &ssc->shadow_segments) {
      if (!(ss->flag & LRT_SHADOW_CASTED)) {
        continue;
      }
      if (!ss->next) {
        break;
      }
      LineartEdge *e = &elist[ei];
      BLI_addtail(&e->segments, &es[ei]);
      LineartVert *v1 = &vlist[ei * 2], *v2 = &vlist[ei * 2 + 1];
      copy_v3_v3_db(v1->gloc, ss->g2);
      copy_v3_v3_db(v2->gloc, ((LineartShadowSegment *)ss->next)->g1);
      e->v1 = v1;
      e->v2 = v2;
      e->t1 = (LineartTriangle *)ssc->e_ref; /* See LineartEdge::t1 for usage. */
      e->t2 = (LineartTriangle *)(ssc->e_ref_light_contour ? ssc->e_ref_light_contour :
                                                             ssc->e_ref);
      e->target_reference = ss->target_reference;
      e->silhouette_group = ss->silhouette_group;
      e->flags = (LRT_EDGE_FLAG_PROJECTED_SHADOW |
                  ((ss->flag & LRT_SHADOW_FACING_LIGHT) ? LRT_EDGE_FLAG_SHADOW_FACING_LIGHT : 0));
      ei++;
    }
    if (do_original_edges) {
      /* Occlusion-corrected light contour. */
      LineartEdge *e = &elist[ei];
      BLI_addtail(&e->segments, &es[ei]);
      LineartVert *v1 = &vlist[ei * 2], *v2 = &vlist[ei * 2 + 1];
      // if (ssc->e_ref->t1 && ssc->e_ref->t2) {
      copy_v3_v3_db(v1->gloc, ssc->g1);
      copy_v3_v3_db(v2->gloc, ssc->g2);
      //}
      e->v1 = v1;
      e->v2 = v2;
      e->t1 = e->t2 = (LineartTriangle *)ssc->e_ref;
      e->flags = LRT_EDGE_FLAG_LIGHT_CONTOUR;
      ei++;
    }
  }
  return true;
}

static void lineart_shadow_silhouette_task(void *__restrict userdata,
                                           const int ssc_index,
                                           const TaskParallelTLS *__restrict UNUSED(tls))
{
  LineartRenderBuffer *rb = (LineartRenderBuffer *)userdata;
  LineartShadowSegmentContainer *ssc = &rb->shadow_containers[ssc_index];

  LineartEdge *e = ssc->e_ref;
  LineartEdgeSegment *es = ssc->es_ref;
  double es_start = es->at, es_end = es->next ? es->next->at : 1.0f;
  LISTBASE_FOREACH (LineartShadowSegment *, ss, &ssc->shadow_segments) {
    if (!(ss->flag & LRT_SHADOW_CASTED)) {
      continue;
    }
    if (!ss->next) {
      break;
    }
    /* If the edge is a mesh border or a material border, it can't be erased with silhouette
     * projection. */
    if ((e->t1 && e->t2 && e->t1->silhouette_group != e->t2->silhouette_group)) {
      continue;
    }
    if (e->silhouette_group != ss->silhouette_group) {
      continue;
    }
    double at_start = interpd(es_end, es_start, ss->at);
    double at_end = interpd(es_end, es_start, ss->next->at);
    lineart_edge_cut(rb, e, at_start, at_end, 0, 0, LRT_SHADOW_SILHOUETTE_ERASED);
  }
}

static void lineart_shadow_register_silhouette(LineartRenderBuffer *rb)
{
  TaskParallelSettings silhouette_settings;
  BLI_parallel_range_settings_defaults(&silhouette_settings);
  /* Set the minimum amount of edges a thread has to process. */
  silhouette_settings.min_iter_per_thread = 2000;

  BLI_task_parallel_range(
      0, rb->shadow_containers_count, rb, lineart_shadow_silhouette_task, &silhouette_settings);
}

typedef struct EnclosedShapesData {
  LineartRenderBuffer *rb;
  LineartRenderBuffer *shadow_rb;
} EnclosedShapesData;

static void lineart_shadow_enclosed_shapes_task(void *__restrict userdata,
                                                const int shadow_rb_edge_index,
                                                const TaskParallelTLS *__restrict UNUSED(tls))
{
  EnclosedShapesData *data = (EnclosedShapesData *)userdata;
  LineartRenderBuffer *rb = data->rb;
  LineartRenderBuffer *shadow_rb = data->shadow_rb;

  LineartEdge *e;
  LineartEdgeSegment *es;
  e = shadow_rb->pending_edges.array[shadow_rb_edge_index];
  /* Only care about shade-on-light and light-on-light situations, hence we only need
   * non-occludded segments in shadow buffer. */
  if (e->min_occ > 0) {
    return;
  }
  for (es = e->segments.first; es; es = es->next) {
    if (es->occlusion > 0) {
      continue;
    }
    double next_at = es->next ? ((LineartEdgeSegment *)es->next)->at : 1.0f;
    LineartEdge *orig_e = (LineartEdge *)e->t2;

    /* Shadow view space to global. */
    double ga1 = e->v1->fbcoord[3] * es->at /
                 (es->at * e->v1->fbcoord[3] + (1 - es->at) * e->v2->fbcoord[3]);
    double ga2 = e->v1->fbcoord[3] * next_at /
                 (next_at * e->v1->fbcoord[3] + (1 - next_at) * e->v2->fbcoord[3]);
    double g1[3], g2[3], g1v[4], g2v[4];
    interp_v3_v3v3_db(g1, e->v1->gloc, e->v2->gloc, ga1);
    interp_v3_v3v3_db(g2, e->v1->gloc, e->v2->gloc, ga2);
    mul_v4_m4v3_db(g1v, rb->view_projection, g1);
    mul_v4_m4v3_db(g2v, rb->view_projection, g2);

    if (rb->cam_is_persp) {
      mul_v3db_db(g1v, (1 / g1v[3]));
      mul_v3db_db(g2v, (1 / g2v[3]));
    }

    g1v[0] -= rb->shift_x * 2;
    g1v[1] -= rb->shift_y * 2;
    g2v[0] -= rb->shift_x * 2;
    g2v[1] -= rb->shift_y * 2;

#define GET_RATIO(n) \
  (fabs(orig_e->v2->fbcoord[0] - orig_e->v1->fbcoord[0]) > \
   fabs(orig_e->v2->fbcoord[1] - orig_e->v1->fbcoord[1])) ? \
      ((g##n##v[0] - orig_e->v1->fbcoord[0]) / \
       (orig_e->v2->fbcoord[0] - orig_e->v1->fbcoord[0])) : \
      ((g##n##v[1] - orig_e->v1->fbcoord[1]) / (orig_e->v2->fbcoord[1] - orig_e->v1->fbcoord[1]))
    double la1, la2;
    la1 = GET_RATIO(1);
    la2 = GET_RATIO(2);
#undef GET_RATIO

    lineart_edge_cut(rb, orig_e, la1, la2, 0, 0, LRT_SHADOW_MASK_ENCLOSED_SHAPE);
  }
}

static void lineart_shadow_register_enclosed_shapes(LineartRenderBuffer *rb,
                                                    LineartRenderBuffer *shadow_rb)
{
  TaskParallelSettings shape_settings;
  BLI_parallel_range_settings_defaults(&shape_settings);
  /* Set the minimum amount of edges a thread has to process. */
  shape_settings.min_iter_per_thread = 2000;

  EnclosedShapesData data = {0};
  data.rb = rb;
  data.shadow_rb = shadow_rb;

  BLI_task_parallel_range(0,
                          shadow_rb->pending_edges.next,
                          &data,
                          lineart_shadow_enclosed_shapes_task,
                          &shape_settings);
}

bool lineart_main_try_generate_shadow(Depsgraph *depsgraph,
                                      Scene *scene,
                                      LineartRenderBuffer *original_rb,
                                      LineartGpencilModifierData *lmd,
                                      LineartStaticMemPool *shadow_data_pool,
                                      LineartElementLinkNode **r_veln,
                                      LineartElementLinkNode **r_eeln,
                                      ListBase *r_calculated_edges_eln_list,
                                      LineartRenderBuffer **r_shadow_rb_if_reproject)
{
  if ((!original_rb->use_shadow && !original_rb->use_light_contour &&
       !original_rb->shadow_selection) ||
      (!lmd->light_contour_object)) {
    return false;
  }

  double t_start;
  if (G.debug_value == 4000) {
    t_start = PIL_check_seconds_timer();
  }

  bool is_persp = true;

  if (lmd->light_contour_object->type == OB_LAMP) {
    Light *la = (Light *)lmd->light_contour_object->data;
    if (la->type == LA_SUN) {
      is_persp = false;
    }
  }

  LineartRenderBuffer *rb = MEM_callocN(sizeof(LineartRenderBuffer),
                                        "LineArt render buffer copied");
  memcpy(rb, original_rb, sizeof(LineartRenderBuffer));

  rb->do_shadow_cast = true;
  rb->shadow_data_pool = shadow_data_pool;

  /* See LineartRenderBuffer::edge_data_pool for explaination. */
  if (rb->shadow_selection) {
    rb->edge_data_pool = shadow_data_pool;
  }
  else {
    rb->edge_data_pool = &rb->render_data_pool;
  }

  copy_v3_v3_db(rb->camera_pos_secondary, rb->camera_pos);
  copy_m4_m4(rb->cam_obmat_secondary, rb->cam_obmat);

  copy_m4_m4(rb->cam_obmat, lmd->light_contour_object->obmat);
  copy_v3db_v3fl(rb->camera_pos, rb->cam_obmat[3]);
  rb->cam_is_persp_secondary = rb->cam_is_persp;
  rb->cam_is_persp = is_persp;
  rb->near_clip = is_persp ? lmd->shadow_camera_near : -lmd->shadow_camera_far;
  rb->far_clip = lmd->shadow_camera_far;
  rb->w = lmd->shadow_camera_size;
  rb->h = lmd->shadow_camera_size;
  /* Need to prevent wrong camera configuration so that shadow computation won't stall. */
  if (!rb->w || !rb->h) {
    rb->w = rb->h = 200;
  }
  if (!rb->near_clip || !rb->far_clip) {
    rb->near_clip = 0.1f;
    rb->far_clip = 200.0f;
  }
  rb->tile_recursive_level = is_persp ? LRT_TILE_RECURSIVE_PERSPECTIVE : LRT_TILE_RECURSIVE_ORTHO;

  /* Contour and loose edge from light viewing direction will be casted as shadow, so only
   * force them on. If we need lit/shaded information for other line types, they are then
   * enabled as-is so that cutting positions can also be calculated through shadow projection.
   */
  if (!rb->shadow_selection) {
    rb->use_crease = rb->use_material = rb->use_edge_marks = rb->use_intersections =
        rb->use_light_contour = false;
  }
  else {
    rb->use_contour_secondary = true;
    rb->allow_duplicated_types = true;
  }
  rb->use_loose = true;
  rb->use_contour = true;

  rb->max_occlusion_level = 0; /* No point getting see-through projections there. */
  rb->use_back_face_culling = false;

  /* Override matrices to light "camera". */
  double proj[4][4], view[4][4], result[4][4];
  float inv[4][4];
  if (is_persp) {
    lineart_matrix_perspective_44d(proj, DEG2RAD(160), 1, rb->near_clip, rb->far_clip);
  }
  else {
    lineart_matrix_ortho_44d(proj, -rb->w, rb->w, -rb->h, rb->h, rb->near_clip, rb->far_clip);
  }
  invert_m4_m4(inv, rb->cam_obmat);
  mul_m4db_m4db_m4fl_uniq(result, proj, inv);
  copy_m4_m4_db(proj, result);
  copy_m4_m4_db(rb->view_projection, proj);
  unit_m4_db(view);
  copy_m4_m4_db(rb->view, view);

  lineart_main_get_view_vector(rb);

  lineart_main_load_geometries(
      depsgraph, scene, NULL, rb, lmd->flags & LRT_ALLOW_DUPLI_OBJECTS, true, NULL);

  if (!rb->vertex_buffer_pointers.first) {
    /* No geometry loaded, return early. */
    lineart_destroy_render_data_keep_init(rb);
    MEM_freeN(rb);
    return false;
  }

  /* The exact same process as in MOD_lineart_compute_feature_lines() until occlusion finishes.
   */

  lineart_main_bounding_area_make_initial(rb);
  lineart_main_cull_triangles(rb, false);
  lineart_main_cull_triangles(rb, true);
  lineart_main_free_adjacent_data(rb);
  lineart_main_perspective_division(rb);
  lineart_main_discard_out_of_frame_edges(rb);
  lineart_main_add_triangles(rb);
  lineart_main_bounding_areas_connect_post(rb);
  lineart_main_link_lines(rb);
  lineart_main_occlusion_begin(rb);

  /* Do shadow cast stuff then get generated vert/edge data. */
  lineart_shadow_cast(rb, true, false);
  bool any_generated = lineart_shadow_cast_generate_edges(rb, true, r_veln, r_eeln);

  if (rb->shadow_selection) {
    memcpy(r_calculated_edges_eln_list, &rb->line_buffer_pointers, sizeof(ListBase));
  }

  if (rb->shadow_enclose_shapes) {
    /* Need loaded data for reprojecting the 3rd time to get shape boundary against lit/shaded
     * region. */
    (*r_shadow_rb_if_reproject) = rb;
  }
  else {
    lineart_destroy_render_data_keep_init(rb);
    MEM_freeN(rb);
  }

  if (G.debug_value == 4000) {
    double t_elapsed = PIL_check_seconds_timer() - t_start;
    printf("Line art shadow stage 1 time: %f\n", t_elapsed);
  }

  return any_generated;
}

typedef struct LineartShadowTransformData {
  LineartRenderBuffer *rb;
  LineartVert *v;
} LineartShadowTransformData;

static void lineart_shadow_transform_to_global_task(void *__restrict userdata,
                                                    const int element_index,
                                                    const TaskParallelTLS *__restrict UNUSED(tls))
{
  LineartShadowTransformData *data = (LineartShadowTransformData *)userdata;
  LineartRenderBuffer *rb = data->rb;
  LineartVert *v = &data->v[element_index];
  mul_v4_m4v3_db(v->fbcoord, rb->view_projection, v->gloc);
}

void lineart_main_transform_and_add_shadow(LineartRenderBuffer *rb,
                                           LineartElementLinkNode *veln,
                                           LineartElementLinkNode *eeln)
{

  TaskParallelSettings transform_settings;
  BLI_parallel_range_settings_defaults(&transform_settings);
  /* Set the minimum amount of edges a thread has to process. */
  transform_settings.min_iter_per_thread = 4000;

  LineartShadowTransformData data = {0};
  data.rb = rb;
  data.v = (LineartVert *)veln->pointer;

  BLI_task_parallel_range(
      0, veln->element_count, &data, lineart_shadow_transform_to_global_task, &transform_settings);

  LineartEdge *e = eeln->pointer;
  for (int i = 0; i < eeln->element_count; i++) {
    lineart_add_edge_to_array(&rb->pending_edges, &e[i]);
  }
  BLI_addtail(&rb->vertex_buffer_pointers, veln);
  BLI_addtail(&rb->line_buffer_pointers, eeln);
}

void lineart_main_make_enclosed_shapes(LineartRenderBuffer *rb, LineartRenderBuffer *shadow_rb)
{
  double t_start;
  if (G.debug_value == 4000) {
    t_start = PIL_check_seconds_timer();
  }

  if (shadow_rb || rb->shadow_use_silhouette) {
    lineart_shadow_cast(rb, false, shadow_rb ? true : false);
    if (rb->shadow_use_silhouette) {
      lineart_shadow_register_silhouette(rb);
    }
  }

  if (G.debug_value == 4000) {
    double t_elapsed = PIL_check_seconds_timer() - t_start;
    printf("Line art shadow stage 2 cast and silhouette time: %f\n", t_elapsed);
  }

  if (!shadow_rb) {
    return;
  }

  rb->shadow_data_pool = &rb->render_data_pool;

  if (shadow_rb->pending_edges.array) {
    MEM_freeN(shadow_rb->pending_edges.array);
    shadow_rb->pending_edges.array = NULL;
    shadow_rb->pending_edges.next = shadow_rb->pending_edges.max = 0;
  }

  LineartElementLinkNode *shadow_veln, *shadow_eeln;

  bool any_generated = lineart_shadow_cast_generate_edges(rb, false, &shadow_veln, &shadow_eeln);

  if (!any_generated) {
    return;
  }

  LineartVert *v = shadow_veln->pointer;
  for (int i = 0; i < shadow_veln->element_count; i++) {
    mul_v4_m4v3_db(v[i].fbcoord, shadow_rb->view_projection, v[i].gloc);
    if (shadow_rb->cam_is_persp) {
      mul_v3db_db(v[i].fbcoord, (1 / v[i].fbcoord[3]));
    }
  }

  lineart_finalize_object_edge_array_reserve(&shadow_rb->pending_edges,
                                             shadow_eeln->element_count);

  LineartEdge *se = shadow_eeln->pointer;
  for (int i = 0; i < shadow_eeln->element_count; i++) {
    lineart_add_edge_to_array(&shadow_rb->pending_edges, &se[i]);
  }

  shadow_rb->scheduled_count = 0;

  lineart_main_clear_linked_edges(shadow_rb);
  lineart_main_link_lines(shadow_rb);
  lineart_main_occlusion_begin(shadow_rb);

  lineart_shadow_register_enclosed_shapes(rb, shadow_rb);

  if (G.debug_value == 4000) {
    double t_elapsed = PIL_check_seconds_timer() - t_start;
    printf("Line art shadow stage 2 total time: %f\n", t_elapsed);
  }
}
