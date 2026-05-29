#include <stdio.h>
#include <stdlib.h>

#include "lpr_special_route.h"

static void expect_route(const char *name, enum lpr_special_route got,
                         enum lpr_special_route want)
{
    if (got != want) {
        fprintf(stderr, "[FAIL] %s got=%d want=%d\n", name, got, want);
        exit(1);
    }
}

static void test_black_embassy_beats_white_text(void)
{
    expect_route("black_embassy",
                 lpr_choose_unknown_plate_route(true, true, true, 0.24f, 0.55f),
                 LPR_SPECIAL_ROUTE_EMBASSY);
}

static void test_white_police_routes_to_police(void)
{
    expect_route("white_police",
                 lpr_choose_unknown_plate_route(true, true, true, 0.62f, 0.06f),
                 LPR_SPECIAL_ROUTE_POLICE);
}

static void test_ambiguous_uses_special_fallback(void)
{
    expect_route("ambiguous_special",
                 lpr_choose_unknown_plate_route(true, true, true, 0.23f, 0.20f),
                 LPR_SPECIAL_ROUTE_SPECIAL);
}

static void test_dark_plate_never_falls_to_police_when_embassy_missing(void)
{
    expect_route("dark_no_embassy",
                 lpr_choose_unknown_plate_route(true, false, true, 0.24f, 0.55f),
                 LPR_SPECIAL_ROUTE_SPECIAL);
}

int main(void)
{
    test_black_embassy_beats_white_text();
    test_white_police_routes_to_police();
    test_ambiguous_uses_special_fallback();
    test_dark_plate_never_falls_to_police_when_embassy_missing();
    printf("[PASS] test_lpr_special_route\n");
    return 0;
}
