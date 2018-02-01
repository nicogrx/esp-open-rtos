# Component makefile for extras/arducam

INC_DIRS += $(arducam_ROOT)..
arducam_SRC_DIR =  $(arducam_ROOT)

$(eval $(call component_compile_rules,arducam))
