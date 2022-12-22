CFLAGS_MODULE += -Werror
obj-m += $(addsuffix .o, $(notdir $(basename $(filter-out %.mod.c, $(wildcard $(BR2_EXTERNAL_USERCONFIGTREE_PATH)/*.c)))))
