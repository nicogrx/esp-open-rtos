# Component makefile for extras/l293d

# expected anyone using this driver includes it as 'l293d/l293d.h'
INC_DIRS += $(l293d_ROOT)..

# args for passing into compile rule generation
l293d_SRC_DIR = $(l293d_ROOT)

$(eval $(call component_compile_rules,l293d))
