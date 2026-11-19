extern "C" {
#include <framework/mlt.h>

mlt_producer producer_osv_init(mlt_profile profile, mlt_service_type type, const char* id, const char* arg);
mlt_filter filter_dual_lens_reframe_init(mlt_profile profile, mlt_service_type type, const char* id, const char* arg);

MLT_REPOSITORY
{
    MLT_REGISTER(mlt_service_producer_type, "osv", producer_osv_init);
    MLT_REGISTER(mlt_service_filter_type, "dual_lens_reframe", filter_dual_lens_reframe_init);
}
}
