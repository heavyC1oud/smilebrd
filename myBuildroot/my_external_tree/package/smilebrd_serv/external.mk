################################################################################
#
# smilebrd_serv
#
################################################################################

SMILEBRD_SERV_VERSION = v2.0
SMILEBRD_SERV_SITE = $(BR2_EXTERNAL_USERCONFIGTREE_PATH)/package/smilebrd_serv
SMILEBRD_SERV_SITE_METHOD = local

$(eval $(kernel-module))
$(eval $(generic-package))
