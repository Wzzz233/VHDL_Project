#ifndef LPR_SPECIAL_ROUTE_H
#define LPR_SPECIAL_ROUTE_H

#include <stdbool.h>

enum lpr_special_route {
    LPR_SPECIAL_ROUTE_BLUE = 0,
    LPR_SPECIAL_ROUTE_SPECIAL,
    LPR_SPECIAL_ROUTE_POLICE,
    LPR_SPECIAL_ROUTE_EMBASSY,
};

static inline bool lpr_special_tone_candidate(float white_ratio, float dark_ratio)
{
    if (white_ratio < 0.0f)
        white_ratio = 0.0f;
    if (dark_ratio < 0.0f)
        dark_ratio = 0.0f;
    if (white_ratio > 1.0f)
        white_ratio = 1.0f;
    if (dark_ratio > 1.0f)
        dark_ratio = 1.0f;
    return white_ratio >= 0.30f || dark_ratio >= 0.35f || (white_ratio + dark_ratio) >= 0.48f;
}

static inline enum lpr_special_route lpr_choose_unknown_plate_route(bool police_enabled,
                                                                    bool embassy_enabled,
                                                                    bool special_enabled,
                                                                    float white_ratio,
                                                                    float dark_ratio)
{
    bool dark_dominant;
    bool white_dominant;

    if (white_ratio < 0.0f)
        white_ratio = 0.0f;
    if (dark_ratio < 0.0f)
        dark_ratio = 0.0f;
    if (white_ratio > 1.0f)
        white_ratio = 1.0f;
    if (dark_ratio > 1.0f)
        dark_ratio = 1.0f;

    dark_dominant = (dark_ratio >= 0.35f && dark_ratio >= white_ratio * 1.15f);
    white_dominant = (white_ratio >= 0.30f && white_ratio >= dark_ratio * 1.25f);

    if (embassy_enabled && dark_dominant)
        return LPR_SPECIAL_ROUTE_EMBASSY;
    if (police_enabled && white_dominant)
        return LPR_SPECIAL_ROUTE_POLICE;
    if (special_enabled)
        return LPR_SPECIAL_ROUTE_SPECIAL;
    return LPR_SPECIAL_ROUTE_BLUE;
}

#endif
