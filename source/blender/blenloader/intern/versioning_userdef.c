/* SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup blenloader
 *
 * Version patch user preferences.
 */
#define DNA_DEPRECATED_ALLOW
#include <string.h>

#include "BLI_listbase.h"
#include "BLI_math.h"
#include "BLI_string.h"
#include "BLI_utildefines.h"

#include "DNA_anim_types.h"
#include "DNA_collection_types.h"
#include "DNA_curve_types.h"
#include "DNA_scene_types.h"
#include "DNA_space_types.h"
#include "DNA_userdef_types.h"
#include "DNA_windowmanager_types.h"

#include "BKE_addon.h"
#include "BKE_blender_version.h"
#include "BKE_colorband.h"
#include "BKE_idprop.h"
#include "BKE_keyconfig.h"
#include "BKE_main.h"
#include "BKE_preferences.h"

#include "BLO_readfile.h"

#include "readfile.h" /* Own include. */

#include "WM_types.h"
#include "wm_event_types.h"

/* Don't use translation strings in versioning!
 * These depend on the preferences already being read.
 * If this is important we can set the translations as part of versioning preferences,
 * however that should only be done if there are important use-cases. */
#if 0
#  include "BLT_translation.h"
#else
#  define N_(msgid) msgid
#endif

/* For versioning we only ever want to manipulate preferences passed in. */
#define U BLI_STATIC_ASSERT(false, "Global 'U' not allowed, only use arguments passed in!")

static void do_versions_theme(const UserDef *userdef, bTheme *btheme)
{

#define USER_VERSION_ATLEAST(ver, subver) MAIN_VERSION_ATLEAST(userdef, ver, subver)
#define FROM_DEFAULT_V4_UCHAR(member) copy_v4_v4_uchar(btheme->member, U_theme_default.member)

  if (!USER_VERSION_ATLEAST(280, 25)) {
    copy_v4_v4_uchar(btheme->space_action.anim_preview_range, btheme->space_action.anim_active);
    copy_v4_v4_uchar(btheme->space_nla.anim_preview_range, btheme->space_nla.anim_active);
    copy_v4_v4_uchar(btheme->space_graph.anim_preview_range, btheme->space_action.anim_active);
  }

  if (!USER_VERSION_ATLEAST(280, 26)) {
    FROM_DEFAULT_V4_UCHAR(tui.icon_collection);
    FROM_DEFAULT_V4_UCHAR(tui.icon_object);
    FROM_DEFAULT_V4_UCHAR(tui.icon_object_data);
    FROM_DEFAULT_V4_UCHAR(tui.icon_modifier);
    FROM_DEFAULT_V4_UCHAR(tui.icon_shading);
  }

  if (!USER_VERSION_ATLEAST(280, 27)) {
    FROM_DEFAULT_V4_UCHAR(space_action.shade2);
    FROM_DEFAULT_V4_UCHAR(space_action.hilite);
    FROM_DEFAULT_V4_UCHAR(space_action.group);
    FROM_DEFAULT_V4_UCHAR(space_action.group_active);
    FROM_DEFAULT_V4_UCHAR(space_action.strip_select);
    FROM_DEFAULT_V4_UCHAR(space_action.ds_channel);
    FROM_DEFAULT_V4_UCHAR(space_action.ds_subchannel);
    FROM_DEFAULT_V4_UCHAR(space_action.keytype_movehold);
    FROM_DEFAULT_V4_UCHAR(space_action.keytype_movehold_select);
  }

  if (!USER_VERSION_ATLEAST(280, 28)) {
    FROM_DEFAULT_V4_UCHAR(space_action.ds_ipoline);
  }

  if (!USER_VERSION_ATLEAST(280, 29)) {
    FROM_DEFAULT_V4_UCHAR(space_properties.navigation_bar);
  }
  if (!USER_VERSION_ATLEAST(280, 31)) {
    FROM_DEFAULT_V4_UCHAR(space_clip.list_text);
  }

  if (!USER_VERSION_ATLEAST(280, 36)) {
    FROM_DEFAULT_V4_UCHAR(tui.wcol_state.inner_changed);
    FROM_DEFAULT_V4_UCHAR(tui.wcol_state.inner_changed_sel);
  }

  if (!USER_VERSION_ATLEAST(280, 39)) {
    FROM_DEFAULT_V4_UCHAR(space_clip.metadatabg);
    FROM_DEFAULT_V4_UCHAR(space_clip.metadatatext);
  }

  if (!USER_VERSION_ATLEAST(280, 40)) {
    FROM_DEFAULT_V4_UCHAR(space_preferences.navigation_bar);
    copy_v4_v4_uchar(btheme->space_preferences.execution_buts,
                     btheme->space_preferences.navigation_bar);
  }

  if (!USER_VERSION_ATLEAST(280, 41)) {
    FROM_DEFAULT_V4_UCHAR(space_view3d.back);
  }

  if (!USER_VERSION_ATLEAST(280, 52)) {
    FROM_DEFAULT_V4_UCHAR(space_info.info_info);
  }

  if (!USER_VERSION_ATLEAST(280, 64)) {
    FROM_DEFAULT_V4_UCHAR(tui.icon_scene);

    if (btheme->space_view3d.obcenter_dia == 0) {
      btheme->space_view3d.obcenter_dia = U_theme_default.space_view3d.obcenter_dia;
    }

    FROM_DEFAULT_V4_UCHAR(space_graph.text);
    FROM_DEFAULT_V4_UCHAR(space_action.text);
    FROM_DEFAULT_V4_UCHAR(space_nla.text);
    FROM_DEFAULT_V4_UCHAR(space_sequencer.text);
    FROM_DEFAULT_V4_UCHAR(space_clip.text);

    FROM_DEFAULT_V4_UCHAR(space_graph.time_scrub_background);
    FROM_DEFAULT_V4_UCHAR(space_action.time_scrub_background);
    FROM_DEFAULT_V4_UCHAR(space_nla.time_scrub_background);
    FROM_DEFAULT_V4_UCHAR(space_sequencer.time_scrub_background);
    FROM_DEFAULT_V4_UCHAR(space_clip.time_scrub_background);
  }

  if (!USER_VERSION_ATLEAST(280, 67)) {
    FROM_DEFAULT_V4_UCHAR(space_outliner.selected_object);
    FROM_DEFAULT_V4_UCHAR(space_outliner.active_object);
    FROM_DEFAULT_V4_UCHAR(space_outliner.edited_object);
    FROM_DEFAULT_V4_UCHAR(space_outliner.row_alternate);
  }

  if (!USER_VERSION_ATLEAST(281, 3)) {
    FROM_DEFAULT_V4_UCHAR(space_outliner.selected_highlight);
    FROM_DEFAULT_V4_UCHAR(space_outliner.active);
  }

  if (!USER_VERSION_ATLEAST(281, 14)) {
    FROM_DEFAULT_V4_UCHAR(space_file.execution_buts);
    FROM_DEFAULT_V4_UCHAR(tui.icon_folder);
    FROM_DEFAULT_V4_UCHAR(space_clip.path_keyframe_before);
    FROM_DEFAULT_V4_UCHAR(space_clip.path_keyframe_after);
    copy_v4_v4_uchar(btheme->space_nla.nla_track, btheme->space_nla.header);
  }

  if (!USER_VERSION_ATLEAST(282, 5)) {
    FROM_DEFAULT_V4_UCHAR(space_sequencer.anim_preview_range);
    FROM_DEFAULT_V4_UCHAR(space_text.line_numbers);
    FROM_DEFAULT_V4_UCHAR(tui.widget_text_cursor);
    FROM_DEFAULT_V4_UCHAR(space_view3d.face_back);
    FROM_DEFAULT_V4_UCHAR(space_view3d.face_front);
  }

  if (!USER_VERSION_ATLEAST(283, 1)) {
    FROM_DEFAULT_V4_UCHAR(space_view3d.bone_locked_weight);
  }

  if (!USER_VERSION_ATLEAST(283, 2)) {
    FROM_DEFAULT_V4_UCHAR(space_info.info_property);
    FROM_DEFAULT_V4_UCHAR(space_info.info_property_text);
    FROM_DEFAULT_V4_UCHAR(space_info.info_operator);
    FROM_DEFAULT_V4_UCHAR(space_info.info_operator_text);
  }

  if (!USER_VERSION_ATLEAST(283, 5)) {
    FROM_DEFAULT_V4_UCHAR(space_graph.time_marker_line);
    FROM_DEFAULT_V4_UCHAR(space_action.time_marker_line);
    FROM_DEFAULT_V4_UCHAR(space_nla.time_marker_line);
    FROM_DEFAULT_V4_UCHAR(space_sequencer.time_marker_line);
    FROM_DEFAULT_V4_UCHAR(space_clip.time_marker_line);
    FROM_DEFAULT_V4_UCHAR(space_graph.time_marker_line_selected);
    FROM_DEFAULT_V4_UCHAR(space_action.time_marker_line_selected);
    FROM_DEFAULT_V4_UCHAR(space_nla.time_marker_line_selected);
    FROM_DEFAULT_V4_UCHAR(space_sequencer.time_marker_line_selected);
    FROM_DEFAULT_V4_UCHAR(space_clip.time_marker_line_selected);
  }

  if (!USER_VERSION_ATLEAST(283, 6)) {
    btheme->space_node.grid_levels = U_theme_default.space_node.grid_levels;
  }

  if (!USER_VERSION_ATLEAST(283, 9)) {
    FROM_DEFAULT_V4_UCHAR(space_info.info_warning);
  }

  if (!USER_VERSION_ATLEAST(283, 10)) {
    FROM_DEFAULT_V4_UCHAR(tui.gizmo_view_align);

    FROM_DEFAULT_V4_UCHAR(space_sequencer.active_strip);
    FROM_DEFAULT_V4_UCHAR(space_sequencer.selected_strip);
    FROM_DEFAULT_V4_UCHAR(space_sequencer.color_strip);
    FROM_DEFAULT_V4_UCHAR(space_sequencer.mask);
  }

  if (!USER_VERSION_ATLEAST(283, 11)) {
    FROM_DEFAULT_V4_UCHAR(tui.transparent_checker_primary);
    FROM_DEFAULT_V4_UCHAR(tui.transparent_checker_secondary);
    btheme->tui.transparent_checker_size = U_theme_default.tui.transparent_checker_size;
  }
  if (!USER_VERSION_ATLEAST(291, 2)) {
    /* The new defaults for the file browser theme are the same as
     * the outliner's, and it's less disruptive to just copy them. */
    copy_v4_v4_uchar(btheme->space_file.back, btheme->space_outliner.back);
    copy_v4_v4_uchar(btheme->space_file.row_alternate, btheme->space_outliner.row_alternate);

    FROM_DEFAULT_V4_UCHAR(space_image.grid);
  }

  if (!USER_VERSION_ATLEAST(291, 3)) {
    for (int i = 0; i < COLLECTION_COLOR_TOT; ++i) {
      FROM_DEFAULT_V4_UCHAR(collection_color[i].color);
    }

    FROM_DEFAULT_V4_UCHAR(space_properties.match);

    /* New grid theme color defaults are the same as the existing background colors,
     * so they are copied to limit disruption. */
    copy_v3_v3_uchar(btheme->space_clip.grid, btheme->space_clip.back);
    btheme->space_clip.grid[3] = 255.0f;

    copy_v3_v3_uchar(btheme->space_node.grid, btheme->space_node.back);
  }

  if (!USER_VERSION_ATLEAST(291, 9)) {
    FROM_DEFAULT_V4_UCHAR(space_graph.vertex_active);
  }

  if (!USER_VERSION_ATLEAST(292, 5)) {
    for (int i = 0; i < COLLECTION_COLOR_TOT; ++i) {
      FROM_DEFAULT_V4_UCHAR(collection_color[i].color);
    }
    FROM_DEFAULT_V4_UCHAR(space_sequencer.row_alternate);
    FROM_DEFAULT_V4_UCHAR(space_node.nodeclass_geometry);
    FROM_DEFAULT_V4_UCHAR(space_node.nodeclass_attribute);
  }

  if (!USER_VERSION_ATLEAST(292, 12)) {
    FROM_DEFAULT_V4_UCHAR(space_node.nodeclass_shader);
  }

  if (!USER_VERSION_ATLEAST(293, 15)) {
    FROM_DEFAULT_V4_UCHAR(space_properties.active);

    FROM_DEFAULT_V4_UCHAR(space_info.info_error);
    FROM_DEFAULT_V4_UCHAR(space_info.info_warning);
    FROM_DEFAULT_V4_UCHAR(space_info.info_info);
    FROM_DEFAULT_V4_UCHAR(space_info.info_debug);
    FROM_DEFAULT_V4_UCHAR(space_info.info_debug_text);
    FROM_DEFAULT_V4_UCHAR(space_info.info_property);
    FROM_DEFAULT_V4_UCHAR(space_info.info_error);
    FROM_DEFAULT_V4_UCHAR(space_info.info_operator);

    btheme->space_spreadsheet = btheme->space_outliner;
  }

  if (!USER_VERSION_ATLEAST(300, 5)) {
    FROM_DEFAULT_V4_UCHAR(space_spreadsheet.active);
    FROM_DEFAULT_V4_UCHAR(space_spreadsheet.list);
    FROM_DEFAULT_V4_UCHAR(space_spreadsheet.list_text);
    FROM_DEFAULT_V4_UCHAR(space_spreadsheet.list_text_hi);
    FROM_DEFAULT_V4_UCHAR(space_spreadsheet.hilite);
    FROM_DEFAULT_V4_UCHAR(space_spreadsheet.selected_highlight);
  }

  if (!USER_VERSION_ATLEAST(300, 15)) {
    copy_v4_uchar(btheme->space_sequencer.grid, 33);
    btheme->space_sequencer.grid[3] = 255;
  }

  if (!USER_VERSION_ATLEAST(300, 30)) {
    FROM_DEFAULT_V4_UCHAR(space_node.wire);
  }

  if (!USER_VERSION_ATLEAST(300, 31)) {
    for (int i = 0; i < SEQUENCE_COLOR_TOT; ++i) {
      FROM_DEFAULT_V4_UCHAR(strip_color[i].color);
    }
  }

  if (!USER_VERSION_ATLEAST(300, 33)) {
    /* Adjust the frame node alpha now that it is used differently. */
    btheme->space_node.movie[3] = U_theme_default.space_node.movie[3];
  }

  if (!USER_VERSION_ATLEAST(300, 34)) {
    btheme->tui.panel_roundness = 0.4f;
  }

  if (!USER_VERSION_ATLEAST(300, 37)) {
    btheme->space_node.dash_alpha = 0.5f;
  }

  if (!USER_VERSION_ATLEAST(300, 39)) {
    FROM_DEFAULT_V4_UCHAR(space_node.grid);
    btheme->space_node.grid_levels = 7;
  }

  if (!USER_VERSION_ATLEAST(300, 41)) {
    memcpy(btheme, &U_theme_default, sizeof(*btheme));
  }

  /* Again reset the theme, but only if stored with an early 3.1 alpha version. Some changes were
   * done in the release branch and then merged into the 3.1 branch (master). So the previous reset
   * wouldn't work for people who saved their preferences with a 3.1 build meanwhile. But we still
   * don't want to reset theme changes stored in the eventual 3.0 release once opened in a 3.1
   * build. */
  if (userdef->versionfile > 300 && !USER_VERSION_ATLEAST(301, 1)) {
    memcpy(btheme, &U_theme_default, sizeof(*btheme));
  }

  if (!USER_VERSION_ATLEAST(301, 2)) {
    FROM_DEFAULT_V4_UCHAR(space_sequencer.mask);
  }

  if (!USER_VERSION_ATLEAST(302, 8)) {
    btheme->space_node.grid_levels = U_theme_default.space_node.grid_levels;
  }

  if (!USER_VERSION_ATLEAST(302, 9)) {
    FROM_DEFAULT_V4_UCHAR(space_sequencer.list);
    FROM_DEFAULT_V4_UCHAR(space_sequencer.list_title);
    FROM_DEFAULT_V4_UCHAR(space_sequencer.list_text);
    FROM_DEFAULT_V4_UCHAR(space_sequencer.list_text_hi);
  }

  /**
   * Versioning code until next subversion bump goes here.
   *
   * \note Be sure to check when bumping the version:
   * - #blo_do_versions_userdef in this file.
   * - "versioning_{BLENDER_VERSION}.c"
   *
   * \note Keep this message at the bottom of the function.
   */
  {
    /* Keep this block, even when empty. */
    btheme->tui.wcol_view_item = U_theme_default.tui.wcol_view_item;
  }

#undef FROM_DEFAULT_V4_UCHAR

#undef USER_VERSION_ATLEAST
}

/** #UserDef.flag */
#define USER_LMOUSESELECT (1 << 14) /* deprecated */

static void do_version_select_mouse(UserDef *userdef, wmKeyMapItem *kmi)
{
  /* Remove select/action mouse from user defined keymaps. */
  enum {
    ACTIONMOUSE = 0x0005,
    SELECTMOUSE = 0x0006,
    EVT_TWEAK_A = 0x5005,
    EVT_TWEAK_S = 0x5006,
  };
  const bool left = (userdef->flag & USER_LMOUSESELECT) != 0;

  switch (kmi->type) {
    case SELECTMOUSE:
      kmi->type = (left) ? LEFTMOUSE : RIGHTMOUSE;
      break;
    case ACTIONMOUSE:
      kmi->type = (left) ? RIGHTMOUSE : LEFTMOUSE;
      break;
    case EVT_TWEAK_S:
      kmi->type = (left) ? LEFTMOUSE : RIGHTMOUSE;
      kmi->val = KM_CLICK_DRAG;
      break;
    case EVT_TWEAK_A:
      kmi->type = (left) ? RIGHTMOUSE : LEFTMOUSE;
      kmi->val = KM_CLICK_DRAG;
      break;
    default:
      break;
  }
}

static bool keymap_item_has_invalid_wm_context_data_path(wmKeyMapItem *kmi,
                                                         void *UNUSED(user_data))
{
  if (STRPREFIX(kmi->idname, "WM_OT_context_") && kmi->properties) {
    IDProperty *idprop = IDP_GetPropertyFromGroup(kmi->properties, "data_path");
    if (idprop && (idprop->type == IDP_STRING) && STRPREFIX(idprop->data.pointer, "(null)")) {
      return true;
    }
  }
  return false;
}

/** Tweak event types have been removed, replace with click-drag. */
static bool keymap_item_update_tweak_event(wmKeyMapItem *kmi, void *UNUSED(user_data))
{
  /* Tweak events for L M R mouse-buttons. */
  enum {
    EVT_TWEAK_L = 0x5002,
    EVT_TWEAK_M = 0x5003,
    EVT_TWEAK_R = 0x5004,
  };
  switch (kmi->type) {
    case EVT_TWEAK_L:
      kmi->type = LEFTMOUSE;
      break;
    case EVT_TWEAK_M:
      kmi->type = MIDDLEMOUSE;
      break;
    case EVT_TWEAK_R:
      kmi->type = RIGHTMOUSE;
      break;
    default:
      kmi->direction = KM_ANY;
      return false;
  }

  if (kmi->val >= KM_DIRECTION_N && kmi->val <= KM_DIRECTION_NW) {
    kmi->direction = kmi->val;
  }
  else {
    kmi->direction = KM_ANY;
  }
  kmi->val = KM_CLICK_DRAG;
  return false;
}

void blo_do_versions_userdef(UserDef *userdef)
{
  /* #UserDef & #Main happen to have the same struct member. */
#define USER_VERSION_ATLEAST(ver, subver) MAIN_VERSION_ATLEAST(userdef, ver, subver)

  /* the UserDef struct is not corrected with do_versions() .... ugh! */
  if (userdef->menuthreshold1 == 0) {
    userdef->menuthreshold1 = 5;
    userdef->menuthreshold2 = 2;
  }
  if (userdef->mixbufsize == 0) {
    userdef->mixbufsize = 2048;
  }
  if (userdef->autokey_mode == 0) {
    /* 'add/replace' but not on */
    userdef->autokey_mode = 2;
  }
  if (userdef->savetime <= 0) {
    userdef->savetime = 1;
    // XXX      error(STRINGIFY(BLENDER_STARTUP_FILE)" is buggy, please consider removing it.\n");
  }
  if (userdef->gizmo_size == 0) {
    userdef->gizmo_size = 75;
    userdef->gizmo_flag |= USER_GIZMO_DRAW;
  }
  if (userdef->pad_rot_angle == 0.0f) {
    userdef->pad_rot_angle = 15.0f;
  }

  /* graph editor - unselected F-Curve visibility */
  if (userdef->fcu_inactive_alpha == 0) {
    userdef->fcu_inactive_alpha = 0.25f;
  }

  if (!USER_VERSION_ATLEAST(192, 0)) {
    strcpy(userdef->sounddir, "/");
  }

  /* patch to set Dupli Armature */
  if (!USER_VERSION_ATLEAST(220, 0)) {
    userdef->dupflag |= USER_DUP_ARM;
  }

  /* added seam, normal color, undo */
  if (!USER_VERSION_ATLEAST(235, 0)) {
    userdef->uiflag |= USER_GLOBALUNDO;
    if (userdef->undosteps == 0) {
      userdef->undosteps = 32;
    }
  }
  if (!USER_VERSION_ATLEAST(236, 0)) {
    /* illegal combo... */
    if (userdef->flag & USER_LMOUSESELECT) {
      userdef->flag &= ~USER_TWOBUTTONMOUSE;
    }
  }
  if (!USER_VERSION_ATLEAST(240, 0)) {
    userdef->uiflag |= USER_PLAINMENUS;
  }
  if (!USER_VERSION_ATLEAST(242, 0)) {
    /* set defaults for 3D View rotating axis indicator */
    /* since size can't be set to 0, this indicates it's not saved in startup.blend */
    if (userdef->rvisize == 0) {
      userdef->rvisize = 15;
      userdef->rvibright = 8;
      userdef->uiflag |= USER_SHOW_GIZMO_NAVIGATE;
    }
  }
  if (!USER_VERSION_ATLEAST(244, 0)) {
    /* set default number of recently-used files (if not set) */
    if (userdef->recent_files == 0) {
      userdef->recent_files = 10;
    }
  }
  if (!USER_VERSION_ATLEAST(245, 3)) {
    if (userdef->coba_weight.tot == 0) {
      BKE_colorband_init(&userdef->coba_weight, true);
    }
  }
  if (!USER_VERSION_ATLEAST(245, 3)) {
    userdef->flag |= USER_ADD_VIEWALIGNED | USER_ADD_EDITMODE;
  }
  if (!USER_VERSION_ATLEAST(250, 0)) {
    /* adjust grease-pencil distances */
    userdef->gp_manhattandist = 1;
    userdef->gp_euclideandist = 2;

    /* adjust default interpolation for new IPO-curves */
    userdef->ipo_new = BEZT_IPO_BEZ;
  }

  if (!USER_VERSION_ATLEAST(250, 3)) {
    /* new audio system */
    if (userdef->audiochannels == 0) {
      userdef->audiochannels = 2;
    }
    if (userdef->audioformat == 0) {
      userdef->audioformat = 0x24;
    }
    if (userdef->audiorate == 0) {
      userdef->audiorate = 48000;
    }
  }

  if (!USER_VERSION_ATLEAST(250, 8)) {
    wmKeyMap *km;

    for (km = userdef->user_keymaps.first; km; km = km->next) {
      if (STREQ(km->idname, "Armature_Sketch")) {
        strcpy(km->idname, "Armature Sketch");
      }
      else if (STREQ(km->idname, "View3D")) {
        strcpy(km->idname, "3D View");
      }
      else if (STREQ(km->idname, "View3D Generic")) {
        strcpy(km->idname, "3D View Generic");
      }
      else if (STREQ(km->idname, "EditMesh")) {
        strcpy(km->idname, "Mesh");
      }
      else if (STREQ(km->idname, "UVEdit")) {
        strcpy(km->idname, "UV Editor");
      }
      else if (STREQ(km->idname, "Animation_Channels")) {
        strcpy(km->idname, "Animation Channels");
      }
      else if (STREQ(km->idname, "GraphEdit Keys")) {
        strcpy(km->idname, "Graph Editor");
      }
      else if (STREQ(km->idname, "GraphEdit Generic")) {
        strcpy(km->idname, "Graph Editor Generic");
      }
      else if (STREQ(km->idname, "Action_Keys")) {
        strcpy(km->idname, "Dopesheet");
      }
      else if (STREQ(km->idname, "NLA Data")) {
        strcpy(km->idname, "NLA Editor");
      }
      else if (STREQ(km->idname, "Node Generic")) {
        strcpy(km->idname, "Node Editor");
      }
      else if (STREQ(km->idname, "Logic Generic")) {
        strcpy(km->idname, "Logic Editor");
      }
      else if (STREQ(km->idname, "File")) {
        strcpy(km->idname, "File Browser");
      }
      else if (STREQ(km->idname, "FileMain")) {
        strcpy(km->idname, "File Browser Main");
      }
      else if (STREQ(km->idname, "FileButtons")) {
        strcpy(km->idname, "File Browser Buttons");
      }
      else if (STREQ(km->idname, "Buttons Generic")) {
        strcpy(km->idname, "Property Editor");
      }
    }
  }

  if (!USER_VERSION_ATLEAST(252, 3)) {
    if (userdef->flag & USER_LMOUSESELECT) {
      userdef->flag &= ~USER_TWOBUTTONMOUSE;
    }
  }
  if (!USER_VERSION_ATLEAST(252, 4)) {
    /* default new handle type is auto handles */
    userdef->keyhandles_new = HD_AUTO;
  }

  if (!USER_VERSION_ATLEAST(257, 0)) {
    /* Clear #AUTOKEY_FLAG_ONLYKEYINGSET flag from user-preferences,
     * so that it doesn't linger around from old configurations like a ghost. */
    userdef->autokey_flag &= ~AUTOKEY_FLAG_ONLYKEYINGSET;
  }

  if (!USER_VERSION_ATLEAST(260, 3)) {
    /* if new keyframes handle default is stuff "auto", make it "auto-clamped" instead
     * was changed in 260 as part of GSoC11, but version patch was wrong
     */
    if (userdef->keyhandles_new == HD_AUTO) {
      userdef->keyhandles_new = HD_AUTO_ANIM;
    }
  }

  if (!USER_VERSION_ATLEAST(267, 0)) {

    /* GL Texture Garbage Collection */
    if (userdef->textimeout == 0) {
      userdef->texcollectrate = 60;
      userdef->textimeout = 120;
    }
    if (userdef->memcachelimit <= 0) {
      userdef->memcachelimit = 32;
    }
    if (userdef->dbl_click_time == 0) {
      userdef->dbl_click_time = 350;
    }
    if (userdef->v2d_min_gridsize == 0) {
      userdef->v2d_min_gridsize = 35;
    }
    if (userdef->widget_unit == 0) {
      userdef->widget_unit = 20;
    }
    if (userdef->anisotropic_filter <= 0) {
      userdef->anisotropic_filter = 1;
    }

    if (userdef->ndof_sensitivity == 0.0f) {
      userdef->ndof_sensitivity = 1.0f;
      userdef->ndof_flag = (NDOF_LOCK_HORIZON | NDOF_SHOULD_PAN | NDOF_SHOULD_ZOOM |
                            NDOF_SHOULD_ROTATE);
    }

    if (userdef->ndof_orbit_sensitivity == 0.0f) {
      userdef->ndof_orbit_sensitivity = userdef->ndof_sensitivity;

      if (!(userdef->flag & USER_TRACKBALL)) {
        userdef->ndof_flag |= NDOF_TURNTABLE;
      }
    }
  }

  if (!USER_VERSION_ATLEAST(269, 4)) {
    userdef->walk_navigation.mouse_speed = 1.0f;
    userdef->walk_navigation.walk_speed = 2.5f; /* m/s */
    userdef->walk_navigation.walk_speed_factor = 5.0f;
    userdef->walk_navigation.view_height = 1.6f;   /* m */
    userdef->walk_navigation.jump_height = 0.4f;   /* m */
    userdef->walk_navigation.teleport_time = 0.2f; /* s */
  }

  if (!USER_VERSION_ATLEAST(271, 5)) {
    userdef->pie_menu_radius = 100;
    userdef->pie_menu_threshold = 12;
    userdef->pie_animation_timeout = 6;
  }

  if (!USER_VERSION_ATLEAST(275, 2)) {
    userdef->ndof_deadzone = 0.1;
  }

  if (!USER_VERSION_ATLEAST(275, 4)) {
    userdef->node_margin = 80;
  }

  if (!USER_VERSION_ATLEAST(278, 6)) {
    /* Clear preference flags for re-use. */
    userdef->flag &= ~(USER_FLAG_NUMINPUT_ADVANCED | USER_FLAG_UNUSED_2 | USER_FLAG_UNUSED_3 |
                       USER_FLAG_UNUSED_6 | USER_FLAG_UNUSED_7 | USER_FLAG_UNUSED_9 |
                       USER_DEVELOPER_UI);
    userdef->uiflag &= ~(USER_HEADER_BOTTOM);
    userdef->transopts &= ~(USER_TR_UNUSED_2 | USER_TR_UNUSED_3 | USER_TR_UNUSED_4 |
                            USER_TR_UNUSED_6 | USER_TR_UNUSED_7);

    userdef->uiflag |= USER_LOCK_CURSOR_ADJUST;
  }

  if (!USER_VERSION_ATLEAST(280, 20)) {
    userdef->gpu_viewport_quality = 0.6f;

    /* Reset theme, old themes will not be compatible with minor version updates from now on. */
    LISTBASE_FOREACH (bTheme *, btheme, &userdef->themes) {
      memcpy(btheme, &U_theme_default, sizeof(*btheme));
    }

    /* Annotations - new layer color
     * Replace anything that used to be set if it looks like was left
     * on the old default (i.e. black), which most users used
     */
    if ((userdef->gpencil_new_layer_col[3] < 0.1f) || (userdef->gpencil_new_layer_col[0] < 0.1f)) {
      /* - New color matches the annotation pencil icon
       * - Non-full alpha looks better!
       */
      ARRAY_SET_ITEMS(userdef->gpencil_new_layer_col, 0.38f, 0.61f, 0.78f, 0.9f);
    }
  }

  if (!USER_VERSION_ATLEAST(280, 31)) {
    /* Remove select/action mouse from user defined keymaps. */
    LISTBASE_FOREACH (wmKeyMap *, keymap, &userdef->user_keymaps) {
      LISTBASE_FOREACH (wmKeyMapDiffItem *, kmdi, &keymap->diff_items) {
        if (kmdi->remove_item) {
          do_version_select_mouse(userdef, kmdi->remove_item);
        }
        if (kmdi->add_item) {
          do_version_select_mouse(userdef, kmdi->add_item);
        }
      }

      LISTBASE_FOREACH (wmKeyMapItem *, kmi, &keymap->items) {
        do_version_select_mouse(userdef, kmi);
      }
    }
  }

  if (!USER_VERSION_ATLEAST(280, 33)) {
    /* Enable GLTF addon by default. */
    BKE_addon_ensure(&userdef->addons, "io_scene_gltf2");

    userdef->pressure_threshold_max = 1.0f;
  }

  if (!USER_VERSION_ATLEAST(280, 35)) {
    /* Preserve RMB select setting after moving to Python and changing default value. */
    if (USER_VERSION_ATLEAST(280, 32) || !(userdef->flag & USER_LMOUSESELECT)) {
      BKE_keyconfig_pref_set_select_mouse(userdef, 1, false);
    }

    userdef->flag &= ~USER_LMOUSESELECT;
  }

  if (!USER_VERSION_ATLEAST(280, 38)) {
    copy_v4_fl4(userdef->light_param[0].vec, -0.580952, 0.228571, 0.781185, 0.0);
    copy_v4_fl4(userdef->light_param[0].col, 0.900000, 0.900000, 0.900000, 1.000000);
    copy_v4_fl4(userdef->light_param[0].spec, 0.318547, 0.318547, 0.318547, 1.000000);
    userdef->light_param[0].flag = 1;
    userdef->light_param[0].smooth = 0.1;

    copy_v4_fl4(userdef->light_param[1].vec, 0.788218, 0.593482, -0.162765, 0.0);
    copy_v4_fl4(userdef->light_param[1].col, 0.267115, 0.269928, 0.358840, 1.000000);
    copy_v4_fl4(userdef->light_param[1].spec, 0.090838, 0.090838, 0.090838, 1.000000);
    userdef->light_param[1].flag = 1;
    userdef->light_param[1].smooth = 0.25;

    copy_v4_fl4(userdef->light_param[2].vec, 0.696472, -0.696472, -0.172785, 0.0);
    copy_v4_fl4(userdef->light_param[2].col, 0.293216, 0.304662, 0.401968, 1.000000);
    copy_v4_fl4(userdef->light_param[2].spec, 0.069399, 0.020331, 0.020331, 1.000000);
    userdef->light_param[2].flag = 1;
    userdef->light_param[2].smooth = 0.4;

    copy_v4_fl4(userdef->light_param[3].vec, 0.021053, -0.989474, 0.143173, 0.0);
    copy_v4_fl4(userdef->light_param[3].col, 0.0, 0.0, 0.0, 1.0);
    copy_v4_fl4(userdef->light_param[3].spec, 0.072234, 0.082253, 0.162642, 1.000000);
    userdef->light_param[3].flag = 1;
    userdef->light_param[3].smooth = 0.7;

    copy_v3_fl3(userdef->light_ambient, 0.025000, 0.025000, 0.025000);

    userdef->flag &= ~(USER_FLAG_UNUSED_4);

    userdef->uiflag &= ~(USER_HEADER_FROM_PREF | USER_UIFLAG_UNUSED_12 | USER_UIFLAG_UNUSED_22);
  }

  if (!USER_VERSION_ATLEAST(280, 41)) {
    if (userdef->pie_tap_timeout == 0) {
      userdef->pie_tap_timeout = 20;
    }
  }

  if (!USER_VERSION_ATLEAST(280, 44)) {
    userdef->uiflag &= ~(USER_UIFLAG_UNUSED_0 | USER_UIFLAG_UNUSED_1);
    userdef->uiflag2 &= ~(USER_UIFLAG2_UNUSED_0);
    userdef->gp_settings &= ~(GP_PAINT_UNUSED_0);
  }

  if (!USER_VERSION_ATLEAST(280, 50)) {
    /* 3ds is no longer enabled by default and not ported yet. */
    BKE_addon_remove_safe(&userdef->addons, "io_scene_3ds");
  }

  if (!USER_VERSION_ATLEAST(280, 51)) {
    userdef->move_threshold = 2;
  }

  if (!USER_VERSION_ATLEAST(280, 58)) {
    if (userdef->image_draw_method != IMAGE_DRAW_METHOD_GLSL) {
      userdef->image_draw_method = IMAGE_DRAW_METHOD_AUTO;
    }
  }

  /* Patch to set dupli light-probes and grease-pencil. */
  if (!USER_VERSION_ATLEAST(280, 58)) {
    userdef->dupflag |= USER_DUP_LIGHTPROBE;
    userdef->dupflag |= USER_DUP_GPENCIL;
  }

  if (!USER_VERSION_ATLEAST(280, 60)) {
    const float GPU_VIEWPORT_QUALITY_FXAA = 0.10f;
    const float GPU_VIEWPORT_QUALITY_TAA8 = 0.25f;
    const float GPU_VIEWPORT_QUALITY_TAA16 = 0.6f;
    const float GPU_VIEWPORT_QUALITY_TAA32 = 0.8f;

    if (userdef->gpu_viewport_quality <= GPU_VIEWPORT_QUALITY_FXAA) {
      userdef->viewport_aa = SCE_DISPLAY_AA_OFF;
    }
    else if (userdef->gpu_viewport_quality <= GPU_VIEWPORT_QUALITY_TAA8) {
      userdef->viewport_aa = SCE_DISPLAY_AA_FXAA;
    }
    else if (userdef->gpu_viewport_quality <= GPU_VIEWPORT_QUALITY_TAA16) {
      userdef->viewport_aa = SCE_DISPLAY_AA_SAMPLES_8;
    }
    else if (userdef->gpu_viewport_quality <= GPU_VIEWPORT_QUALITY_TAA32) {
      userdef->viewport_aa = SCE_DISPLAY_AA_SAMPLES_16;
    }
    else {
      userdef->viewport_aa = SCE_DISPLAY_AA_SAMPLES_32;
    }
  }

  if (!USER_VERSION_ATLEAST(280, 62)) {
    if (userdef->vbotimeout == 0) {
      userdef->vbocollectrate = 60;
      userdef->vbotimeout = 120;
    }

    if (userdef->lookdev_sphere_size == 0) {
      userdef->lookdev_sphere_size = 150;
    }

    userdef->pref_flag |= USER_PREF_FLAG_SAVE;
  }

  if (!USER_VERSION_ATLEAST(280, 73)) {
    userdef->drag_threshold = 30;
    userdef->drag_threshold_mouse = 3;
    userdef->drag_threshold_tablet = 10;
  }

  if (!USER_VERSION_ATLEAST(281, 9)) {
    /* X3D is no longer enabled by default. */
    BKE_addon_remove_safe(&userdef->addons, "io_scene_x3d");
  }

  if (!USER_VERSION_ATLEAST(281, 12)) {
    userdef->render_display_type = USER_RENDER_DISPLAY_WINDOW;
    userdef->filebrowser_display_type = USER_TEMP_SPACE_DISPLAY_WINDOW;
  }

  if (!USER_VERSION_ATLEAST(281, 13)) {
    userdef->auto_smoothing_new = FCURVE_SMOOTH_CONT_ACCEL;

    if (userdef->file_space_data.display_type == FILE_DEFAULTDISPLAY) {
      memcpy(
          &userdef->file_space_data, &U_default.file_space_data, sizeof(userdef->file_space_data));
    }
  }

  if (!USER_VERSION_ATLEAST(281, 16)) {
    BKE_keyconfig_pref_filter_items(userdef,
                                    &((struct wmKeyConfigFilterItemParams){
                                        .check_item = true,
                                        .check_diff_item_add = true,
                                    }),
                                    keymap_item_has_invalid_wm_context_data_path,
                                    NULL);
  }

  if (!USER_VERSION_ATLEAST(282, 1)) {
    userdef->file_space_data.filter_id = U_default.file_space_data.filter_id;
  }

  if (!USER_VERSION_ATLEAST(282, 4)) {
    if (userdef->view_rotate_sensitivity_turntable == 0.0f) {
      userdef->view_rotate_sensitivity_turntable = DEG2RADF(0.4f);
      userdef->view_rotate_sensitivity_trackball = 1.0f;
    }
    if (userdef->scrollback == 0) {
      userdef->scrollback = U_default.scrollback;
    }

    /* Enable Overlay Engine Smooth Wire by default */
    userdef->gpu_flag |= USER_GPU_FLAG_OVERLAY_SMOOTH_WIRE;
  }

  if (!USER_VERSION_ATLEAST(283, 13)) {
    /* If Translations is off then language should default to English. */
    if ((userdef->transopts & USER_DOTRANSLATE_DEPRECATED) == 0) {
      userdef->language = ULANGUAGE_ENGLISH;
    }
    /* Clear this deprecated flag. */
    userdef->transopts &= ~USER_DOTRANSLATE_DEPRECATED;
  }

  if (!USER_VERSION_ATLEAST(290, 7)) {
    userdef->statusbar_flag = STATUSBAR_SHOW_VERSION;
  }

  if (!USER_VERSION_ATLEAST(291, 1)) {
    if (userdef->collection_instance_empty_size == 0) {
      userdef->collection_instance_empty_size = 1.0f;
    }
  }

  if (!USER_VERSION_ATLEAST(292, 3)) {
    if (userdef->pixelsize == 0.0f) {
      userdef->pixelsize = 1.0f;
    }
    /* Clear old userdef flag for "Camera Parent Lock". */
    userdef->uiflag &= ~USER_UIFLAG_UNUSED_3;
  }

  if (!USER_VERSION_ATLEAST(292, 9)) {
    if (BLI_listbase_is_empty(&userdef->asset_libraries)) {
      BKE_preferences_asset_library_default_add(userdef);
    }
  }

  if (!USER_VERSION_ATLEAST(293, 1)) {
    /* This rename was made after 2.93.0, harmless to run when it's not needed. */
    const char *replace_table[][2] = {
        {"blender", "Blender"},
        {"blender_27x", "Blender_27x"},
        {"industry_compatible", "Industry_Compatible"},
    };
    const int replace_table_len = ARRAY_SIZE(replace_table);

    BLI_str_replace_table_exact(
        userdef->keyconfigstr, sizeof(userdef->keyconfigstr), replace_table, replace_table_len);
    LISTBASE_FOREACH (wmKeyConfigPref *, kpt, &userdef->user_keyconfig_prefs) {
      BLI_str_replace_table_exact(
          kpt->idname, sizeof(kpt->idname), replace_table, replace_table_len);
    }
  }

  if (!USER_VERSION_ATLEAST(293, 12)) {
    if (userdef->gizmo_size_navigate_v3d == 0) {
      userdef->gizmo_size_navigate_v3d = 80;
    }

    userdef->sequencer_proxy_setup = USER_SEQ_PROXY_SETUP_AUTOMATIC;
  }

  if (!USER_VERSION_ATLEAST(293, 13)) {
    BKE_addon_ensure(&userdef->addons, "pose_library");
  }

  if (!USER_VERSION_ATLEAST(300, 21)) {
    /* Deprecated userdef->flag USER_SAVE_PREVIEWS */
    userdef->file_preview_type = (userdef->flag & USER_FLAG_UNUSED_5) ? USER_FILE_PREVIEW_AUTO :
                                                                        USER_FILE_PREVIEW_NONE;
    /* Clear for reuse. */
    userdef->flag &= ~USER_FLAG_UNUSED_5;
  }

  if (!USER_VERSION_ATLEAST(300, 38)) {
    /* Patch to set Dupli Lattice/Camera/Speaker. */
    userdef->dupflag |= USER_DUP_LATTICE;
    userdef->dupflag |= USER_DUP_CAMERA;
    userdef->dupflag |= USER_DUP_SPEAKER;
  }

  if (!USER_VERSION_ATLEAST(300, 40)) {
    /* Rename the default asset library from "Default" to "User Library". This isn't bullet proof
     * since it doesn't handle translations and ignores user changes. But this was an alpha build
     * (experimental) feature and the name is just for display in the UI anyway. So it doesn't have
     * to work perfectly at all. */
    LISTBASE_FOREACH (bUserAssetLibrary *, asset_library, &userdef->asset_libraries) {
      /* Ignores translations, since that would depend on the current preferences (global `U`). */
      if (STREQ(asset_library->name, "Default")) {
        BKE_preferences_asset_library_name_set(
            userdef, asset_library, BKE_PREFS_ASSET_LIBRARY_DEFAULT_NAME);
      }
    }
  }

  if (!USER_VERSION_ATLEAST(300, 40)) {
    LISTBASE_FOREACH (uiStyle *, style, &userdef->uistyles) {
      const int default_title_points = 11; /* UI_DEFAULT_TITLE_POINTS */
      style->paneltitle.points = default_title_points;
      style->grouplabel.points = default_title_points;
    }
  }

  if (!USER_VERSION_ATLEAST(300, 43)) {
    userdef->ndof_flag |= NDOF_CAMERA_PAN_ZOOM;
  }

  if (!USER_VERSION_ATLEAST(302, 5)) {
    BKE_keyconfig_pref_filter_items(userdef,
                                    &((struct wmKeyConfigFilterItemParams){
                                        .check_item = true,
                                        .check_diff_item_add = true,
                                    }),
                                    keymap_item_update_tweak_event,
                                    NULL);
  }

  if (!USER_VERSION_ATLEAST(302, 11)) {
    userdef->dupflag |= USER_DUP_CURVES | USER_DUP_POINTCLOUD;
  }

  /**
   * Versioning code until next subversion bump goes here.
   *
   * \note Be sure to check when bumping the version:
   * - #do_versions_theme in this file.
   * - "versioning_{BLENDER_VERSION}.c"
   *
   * \note Keep this message at the bottom of the function.
   */
  {
    /* Keep this block, even when empty. */
  }

  LISTBASE_FOREACH (bTheme *, btheme, &userdef->themes) {
    do_versions_theme(userdef, btheme);
  }
#undef USER_VERSION_ATLEAST
}

void BLO_sanitize_experimental_features_userpref_blend(UserDef *userdef)
{
  /* User preference experimental settings are only supported in alpha builds.
   * This prevents users corrupting data and relying on API that may change.
   *
   * If user preferences are saved this will be stored in disk as expected.
   * This only starts to take effect when there is a release branch (on beta).
   *
   * At that time master already has its version bumped so its user preferences
   * are not touched by these settings. */

  if (BKE_blender_version_is_alpha()) {
    return;
  }

  MEMSET_STRUCT_AFTER(&userdef->experimental, 0, SANITIZE_AFTER_HERE);
}

#undef USER_LMOUSESELECT
