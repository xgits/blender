# SPDX-License-Identifier: GPL-2.0-or-later

set(INC
  ../include
  ../../blenfont
  ../../blenkernel
  ../../blenlib
  ../../blenloader
  ../../blentranslation
  ../../depsgraph
  ../../draw
  ../../functions
  ../../gpu
  ../../imbuf
  ../../makesdna
  ../../makesrna
  ../../nodes
  ../../python
  ../../render
  ../../windowmanager
  ../../../../intern/glew-mx
  ../../../../intern/guardedalloc
  # RNA_prototypes.h
  ${CMAKE_BINARY_DIR}/source/blender/makesrna
)

set(SRC
  grid_view.cc
  interface.cc
  interface_align.c
  interface_anim.c
  interface_button_group.c
  interface_context_menu.c
  interface_context_path.cc
  interface_drag.cc
  interface_draw.c
  interface_dropboxes.cc
  interface_eyedropper.c
  interface_eyedropper_color.c
  interface_eyedropper_colorband.c
  interface_eyedropper_datablock.c
  interface_eyedropper_depth.c
  interface_eyedropper_driver.c
  interface_eyedropper_gpencil_color.c
  interface_handlers.c
  interface_icons.c
  interface_icons_event.c
  interface_layout.c
  interface_ops.c
  interface_panel.c
  interface_query.cc
  interface_region_color_picker.cc
  interface_region_hud.cc
  interface_region_menu_pie.cc
  interface_region_menu_popup.cc
  interface_region_popover.cc
  interface_region_popup.cc
  interface_region_search.cc
  interface_region_tooltip.c
  interface_regions.cc
  interface_style.cc
  interface_template_asset_view.cc
  interface_template_attribute_search.cc
  interface_template_list.cc
  interface_template_search_menu.cc
  interface_template_search_operator.c
  interface_templates.c
  interface_undo.c
  interface_utils.cc
  interface_view.cc
  interface_widgets.c
  resources.c
  tree_view.cc
  view2d.cc
  view2d_draw.cc
  view2d_edge_pan.cc
  view2d_gizmo_navigate.cc
  view2d_ops.cc

  interface_eyedropper_intern.h
  interface_intern.h
  interface_regions_intern.h
)

set(LIB
  bf_blenkernel
  bf_blenlib
  bf_editor_datafiles
)

if(WITH_INTERNATIONAL)
  add_definitions(-DWITH_INTERNATIONAL)
endif()

if(WITH_HEADLESS)
  add_definitions(-DWITH_HEADLESS)
endif()

if(WITH_PYTHON)
  add_definitions(-DWITH_PYTHON)
endif()

if(WIN32 OR APPLE)
  if(WITH_INPUT_IME)
    add_definitions(-DWITH_INPUT_IME)
  endif()
endif()


blender_add_lib(bf_editor_interface "${SRC}" "${INC}" "${INC_SYS}" "${LIB}")

# RNA_prototypes.h
add_dependencies(bf_editor_interface bf_rna)
