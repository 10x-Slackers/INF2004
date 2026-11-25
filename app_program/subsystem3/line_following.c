/** @file line_following.c
 *
 * @brief Line Following API and task, the only task that reads the line
 * sensors.
 *
 * Both sensors white means centred or lost, so the line counts as lost
 * only after LINE_FOLLOWING_LOST_MM of all-white travel. The task then
 * pivots towards last_side, then back past the start heading the other
 * way, and resumes following as soon as either sensor sees black. A false
 * alarm on a straight costs a small pivot: the line soon reaches a sensor.
 *
 * A sensor stuck on black would hold the turn for ever and never look
 * lost, so one sensor black for over LINE_FOLLOWING_STUCK_MM stops the car
 * as a failed search does, with APP_FAULT_LINE_SENSOR.
 *
 * Barcodes lie on straights, so while the barcode sensor is over one the car
 * holds the heading it had when the barcode was recognised (the wobble hold
 * heading if holding), and a sensor hit turns it at most TURN_MIN. It also
 * slows to CODE_MM_S, so the narrow bars read black. All white
 * is counted from the start again where the barcode ends.
 */

#include "line_following.h"
#include "barcode_decoder.h"
#include "subsystem3.h"
#include "../subsystem2/motion_control.h"

#define LINE_FOLLOWING_BOTH     (DRV_LINE_LEFT | DRV_LINE_RIGHT)
#define LINE_FOLLOWING_JUNCTION 6u /* Samples: 60 ms, 9 mm at 0.15 m/s. */
/* Wheel speed change, permille of the speed: 1000 stops the inner wheel.
 * A sensor turning black starts it at once at TURN_MIN towards that side;
 * it rises to TURN_MAX over RISE_MM while the sensor stays black and falls
 * back to 0 over FALL_MM once neither (or both) is. All white past DRIFT_MM,
 * it turns gently towards last_side, reaching TURN_MIN at LOST_MM. */
#define LINE_FOLLOWING_TURN_MIN   300
#define LINE_FOLLOWING_TURN_MAX   1000
#define LINE_FOLLOWING_RISE_MM    30
#define LINE_FOLLOWING_FALL_MM    10
#define LINE_FOLLOWING_DRIFT_MM   40
#define LINE_FOLLOWING_LOST_MM    80u /* All white this far: lost. */
#define LINE_FOLLOWING_SEARCH_DEG 135 /* Pivot each side of the heading. */
#define LINE_FOLLOWING_PIVOT_MM_S 100 /* Wheel speed while pivoting. */
/* Samples: 6 s. A whole search is under 5 s; a wrong heading never ends. */
#define LINE_FOLLOWING_SEARCH_MAX 600u
#define LINE_FOLLOWING_STUCK_MM   300u /* One sensor black this far: stuck. */
/* Wobble: WOBBLE_HITS sensor hits in a row, alternating sides, each within
 * WOBBLE_MM of the last (any distance while holding), mean a straight. Hits
 * are still steered as usual, but while both sensors are white the car holds
 * the heading halfway between the last left and right hits instead of
 * drifting, turning HOLD_GAIN permille per degree off it, at most TURN_MIN.
 * A curve ends the hold: two hits in a row on one side, one over
 * HOLD_EXIT_MM, a new halfway heading over HOLD_BEND_DEG from the held one,
 * or the car over HOLD_OFF_DEG off it. All white is lost only after
 * HOLD_LOST_MM, over the white gaps of a weave (up to 80 mm) but short, as
 * a held car that lost the line on a curve drives straight on. */
#define LINE_FOLLOWING_WOBBLE_HITS  3u
#define LINE_FOLLOWING_WOBBLE_MM    150u
#define LINE_FOLLOWING_HOLD_GAIN    50
#define LINE_FOLLOWING_HOLD_EXIT_MM 40u
#define LINE_FOLLOWING_HOLD_LOST_MM 120u
#define LINE_FOLLOWING_CODE_LOST_MM 300u /* Over a barcode, 141 mm long. */
/* Over a barcode, at most this fast, so the narrow bars read black. */
#define LINE_FOLLOWING_CODE_MM_S     80u
#define LINE_FOLLOWING_HOLD_BEND_DEG 8
#define LINE_FOLLOWING_HOLD_OFF_DEG  15

/* Line search and stuck sensor check, under gh_mutex. */
typedef struct
{
    unsigned int from_mm : 24;  /* Odometer where all white began. */
    unsigned int phase : 2;     /* 0 none, 1 towards last_side, 2 back. */
    unsigned int start_deg : 9; /* Heading when the search began. */
    unsigned int samples : 10;  /* Since the search began, to SEARCH_MAX. */
    unsigned int black : 2;     /* DRV_LINE_... black ever since black_mm. */
    unsigned int black_mm : 24; /* Odometer when their black began. */
} line_following_search_t;

/* Steering, under gh_mutex. */
typedef struct
{
    signed int turn : 11;        /* Permille of the speed, +left: +-1000. */
    unsigned int speed_mm_s : 9; /* Following speed, 1..500. */
    unsigned int from_mm : 24;   /* Odometer at the last steer. */
} line_following_steer_t;

/* Wobble detection and heading hold, under gh_mutex. */
typedef struct
{
    unsigned int hit_mm : 24;   /* Odometer where the last hit began. */
    unsigned int hits : 2;      /* Alternating hits in a row, up to 3. */
    unsigned int side : 2;      /* DRV_LINE_LEFT or _RIGHT: last hit. */
    bool b_hold : 1;            /* Holding hold_deg. */
    unsigned int hold_deg : 9;  /* Heading held. */
    unsigned int left_deg : 9;  /* Heading at the last left hit. */
    unsigned int right_deg : 9; /* And at the last right hit. */
} line_following_wobble_t;

#if (LINE_FOLLOWING_MAX_MM_S > 511u) || (LINE_FOLLOWING_TURN_MAX > 1023)
#error "g_steer.speed_mm_s or g_steer.turn is too narrow"
#endif
#if LINE_FOLLOWING_SEARCH_MAX > 1023u
#error "g_search.samples is too narrow"
#endif

static ID gh_mutex = 0;
static line_following_t g_line = {0};
static line_following_search_t g_search = {0};
static line_following_steer_t g_steer = {0};
static line_following_wobble_t g_wobble = {0};

static void line_following_stuck(uint32_t bits, uint32_t distance_mm);
static void line_following_watch(uint32_t bits,
                                 const motion_control_status_t * p_motion,
                                 uint32_t lost_mm);
static void line_following_drive(uint32_t bits,
                                 const motion_control_status_t * p_motion,
                                 bool b_straight, uint32_t straight_deg);
static void line_following_steer(uint32_t bits, uint32_t distance_mm,
                                 int32_t max, int32_t speed);
static void line_following_search(const motion_control_status_t * p_motion);
static bool line_following_wobble(uint32_t bits, uint32_t prev,
                                  const motion_control_status_t * p_motion);
static int32_t line_following_off(uint32_t to_deg, uint32_t from_deg);
static void line_following_hold(const motion_control_status_t * p_motion,
                                uint32_t to_deg, int32_t speed);

/*!
 * @brief Create the lock and set up the sensors.
 *
 * @return True if the lock was created.
 */
bool line_following_init(void)
{
    T_CMTX mutex = {0};

    mutex.mtxatr = TA_INHERIT;
    gh_mutex = tk_cre_mtx(&mutex);
    drv_line_init();
    return (gh_mutex > 0);
}

/*!
 * @brief Copy the latest line state.
 */
void line_following_get(line_following_t * p_line)
{
    (void)tk_loc_mtx(gh_mutex, TMO_FEVR);
    *p_line = g_line;
    (void)tk_unl_mtx(gh_mutex);
}

/*!
 * @brief Start steering the car along the line, and clear b_lost and
 * APP_FAULT_LINE_SENSOR.
 *
 * @param[in] speed_mm_s Forward speed, 1 to LINE_FOLLOWING_MAX_MM_S.
 * @return False if the speed is out of range; nothing changes.
 */
bool line_following_start(uint32_t speed_mm_s)
{
    bool const b_valid =
        (speed_mm_s > 0u) && (speed_mm_s <= LINE_FOLLOWING_MAX_MM_S);

    if (b_valid)
    {
        (void)tk_loc_mtx(gh_mutex, TMO_FEVR);
        g_steer.speed_mm_s = speed_mm_s; /* <= MAX_MM_S: fits. */
        g_line.b_following = true;
        g_line.b_lost = false;
        app_fault_clear(APP_FAULT_LINE_SENSOR);
        g_wobble.b_hold = false;
        g_wobble.hits = 0u;
        (void)tk_unl_mtx(gh_mutex);
    }
    return b_valid;
}

/*!
 * @brief Stop steering and brake. The task drives only under the lock, so
 * no drive command can follow the brake.
 */
void line_following_stop(void)
{
    (void)tk_loc_mtx(gh_mutex, TMO_FEVR);
    g_line.b_following = false;
    (void)tk_unl_mtx(gh_mutex);
    motion_control_stop();
}

/*!
 * @brief Line Following task: sample both sensors every 10 ms and, while
 * following, steer along the line or search for it once lost.
 */
void line_following_task(INT stacd, void * p_exinf)
{
    SYSTIM now = {0};
    motion_control_status_t motion = {0};
    barcode_decoder_t barcode = {0};
    uint32_t bits = 0u;
    uint32_t both = 0u; /* Samples in a row with both sensors black. */
    uint32_t prev = 0u; /* Sensors over black at the last sample. */
    bool b_hold = false;
    bool b_straight = false;    /* Over a barcode: hold straight_deg. */
    uint32_t straight_deg = 0u; /* Heading held over a barcode. */

    (void)stacd;
    (void)p_exinf;
    for (;;)
    {
        (void)tk_slp_tsk(TMO_FEVR); /* Woken every 10 ms (subsystem3.c). */
        bits = drv_line_read();
        both = (LINE_FOLLOWING_BOTH == bits) ? (both + 1u) : 0u;
        (void)tk_get_otm(&now);
        motion_control_get_status(&motion);
        barcode_decoder_get(&barcode);

        (void)tk_loc_mtx(gh_mutex, TMO_FEVR);
        prev = g_line.bits;
        g_line.bits = bits;
        line_following_stuck(bits, motion.distance_mm);
        if ((DRV_LINE_LEFT == bits) || (DRV_LINE_RIGHT == bits))
        {
            g_line.last_side = bits;
        }
        g_line.b_junction = (both >= LINE_FOLLOWING_JUNCTION);
        g_line.count++;
        g_line.sample_ms = now.lo;
        b_hold = (g_line.b_following) && (0u == g_search.phase) &&
                 (line_following_wobble(bits, prev, &motion));
        if ((barcode.b_reading) && (!b_straight))
        {
            straight_deg =
                g_wobble.b_hold ? g_wobble.hold_deg : motion.heading_deg;
        }
        else if ((!barcode.b_reading) && b_straight)
        {
            /* The barcode has ended and the shorter limits apply: all white
             * counts from here, or the distance held straight over it
             * would look lost at once. */
            g_search.from_mm = motion.distance_mm;
        }
        else
        {
            /* Still over a barcode, or still not. */
        }
        b_straight = (barcode.b_reading);
        line_following_watch(bits, &motion,
                             b_straight ? LINE_FOLLOWING_CODE_LOST_MM
                             : b_hold   ? LINE_FOLLOWING_HOLD_LOST_MM
                                        : LINE_FOLLOWING_LOST_MM);
        line_following_drive(bits, &motion, b_straight, straight_deg);
        g_line.b_searching = (g_line.b_following) && (0u != g_search.phase);
        (void)tk_unl_mtx(gh_mutex);
    }
}

/*!
 * @brief Stop, as for a lost line, once one sensor has been black for over
 * LINE_FOLLOWING_STUCK_MM: a stuck sensor. Call under gh_mutex.
 *
 * @param[in] bits        Sensors over black.
 * @param[in] distance_mm Odometer now.
 */
static void line_following_stuck(uint32_t bits, uint32_t distance_mm)
{
    g_search.black &= bits; /* Sensors black ever since black_mm. */
    if ((!g_line.b_following) || (0u == g_search.black))
    {
        g_search.black = bits; /* None is: start again from here. */
        g_search.black_mm = distance_mm;
    }
    if (motion_control_mm_since(g_search.black_mm, distance_mm) >
        LINE_FOLLOWING_STUCK_MM)
    {
        g_line.b_following = false;
        g_line.b_lost = true;
        motion_control_stop();
        app_fault_set(APP_FAULT_LINE_SENSOR);
    }
}

/*!
 * @brief Start the search once the car has rolled lost_mm with both
 * sensors white. Call under gh_mutex.
 *
 * @param[in] bits     Sensors over black.
 * @param[in] p_motion Motion status, for the odometer and heading.
 * @param[in] lost_mm  All white this far: lost.
 */
static void line_following_watch(uint32_t bits,
                                 const motion_control_status_t * p_motion,
                                 uint32_t lost_mm)
{
    if ((!g_line.b_following) || (0u != bits))
    {
        g_search.phase = 0u; /* Not following, found, or never lost. */
        g_search.from_mm = p_motion->distance_mm;
    }
    else if ((0u == g_search.phase) &&
             (motion_control_mm_since(g_search.from_mm,
                                      p_motion->distance_mm) >= lost_mm))
    {
        g_search.phase = 1u;
        g_search.start_deg = p_motion->heading_deg;
        g_search.samples = 0u;
        g_wobble.b_hold = false;
        g_wobble.hits = 0u;
    }
    else
    {
        /* Centred, or still searching. */
    }
}

/*!
 * @brief Drive for this sample: hold a heading, steer along the line or
 * search for it. Drives nothing unless following. Call under gh_mutex.
 *
 * @param[in] bits         Sensors over black.
 * @param[in] p_motion     Motion status, for the odometer and heading.
 * @param[in] b_straight   Over a barcode: hold straight_deg, and slow to
 *                         LINE_FOLLOWING_CODE_MM_S.
 * @param[in] straight_deg Heading held over a barcode.
 */
static void line_following_drive(uint32_t bits,
                                 const motion_control_status_t * p_motion,
                                 bool b_straight, uint32_t straight_deg)
{
    /* Both are at most 500: the casts keep them. */
    int32_t const speed = (int32_t)((b_straight && (g_steer.speed_mm_s >
                                                    LINE_FOLLOWING_CODE_MM_S))
                                        ? LINE_FOLLOWING_CODE_MM_S
                                        : g_steer.speed_mm_s);

    if ((g_line.b_following) && (0u == g_search.phase) &&
        (b_straight || (g_wobble.b_hold)) && (0u == bits))
    {
        line_following_hold(
            p_motion, b_straight ? straight_deg : g_wobble.hold_deg, speed);
    }
    else if ((g_line.b_following) && (0u == g_search.phase))
    {
        line_following_steer(bits, p_motion->distance_mm,
                             b_straight ? LINE_FOLLOWING_TURN_MIN
                                        : LINE_FOLLOWING_TURN_MAX,
                             speed);
    }
    else
    {
        g_steer.turn = 0; /* Steering starts again from straight on. */
        g_steer.from_mm = p_motion->distance_mm;
        if (g_line.b_following)
        {
            line_following_search(p_motion);
        }
    }
}

/*!
 * @brief Steer towards the side whose sensor sees black, the turn starting
 * small, rising the further the car rolls with it black and falling once
 * it is not; all white for long, drift back towards the line.
 * Call under gh_mutex while following.
 *
 * @param[in] bits        Sensors over black.
 * @param[in] distance_mm Odometer now.
 * @param[in] max         Largest turn towards a sensor, TURN_MIN..TURN_MAX.
 * @param[in] speed       Forward speed, mm/s.
 */
static void line_following_steer(uint32_t bits, uint32_t distance_mm,
                                 int32_t max, int32_t speed)
{
    int32_t turn = g_steer.turn;
    /* A few mm since the last sample, so the cast keeps it. */
    int32_t const step_mm =
        (int32_t)motion_control_mm_since(g_steer.from_mm, distance_mm);
    /* All white beyond DRIFT_MM; the search starts before it gets large. */
    int32_t const white =
        (int32_t)motion_control_mm_since(g_search.from_mm, distance_mm) -
        LINE_FOLLOWING_DRIFT_MM;
    bool const b_one = (DRV_LINE_LEFT == bits) || (DRV_LINE_RIGHT == bits);
    int32_t target = 0; /* Centred, or a junction: straight on. */
    int32_t step = (step_mm * LINE_FOLLOWING_TURN_MAX) / LINE_FOLLOWING_FALL_MM;

    if (DRV_LINE_LEFT == bits)
    {
        target = max;
    }
    else if (DRV_LINE_RIGHT == bits)
    {
        target = -max;
    }
    else if ((0u == bits) && (white > 0) && (0u != g_line.last_side))
    {
        /* Centred for long, or the line curved away: drift towards it. */
        /* (int32_t): LOST_MM is a small positive constant. */
        target = (white * LINE_FOLLOWING_TURN_MIN) /
                 ((int32_t)LINE_FOLLOWING_LOST_MM - LINE_FOLLOWING_DRIFT_MM);
        target = (DRV_LINE_RIGHT == g_line.last_side) ? -target : target;
    }
    else
    {
        /* Straight on. */
    }
    if (b_one)
    {
        step = (step_mm * (LINE_FOLLOWING_TURN_MAX - LINE_FOLLOWING_TURN_MIN)) /
               LINE_FOLLOWING_RISE_MM;
        /* Below the min, or the other way: start at the min towards it. */
        if (((target * turn) / max) < LINE_FOLLOWING_TURN_MIN)
        {
            turn = (target * LINE_FOLLOWING_TURN_MIN) / max;
        }
    }
    if (turn < target)
    {
        turn = ((turn + step) < target) ? (turn + step) : target;
    }
    else
    {
        turn = ((turn - step) > target) ? (turn - step) : target;
    }
    g_steer.turn = turn; /* Between the old turn and target: fits. */
    g_steer.from_mm = distance_mm;
    step = (speed * turn) / 1000; /* Wheel speed change, +left. */
    (void)motion_control_drive(speed - step, speed + step);
}

/*!
 * @brief Pivot towards last_side up to LINE_FOLLOWING_SEARCH_DEG, then the
 * other way to as far past the start heading; then, or after
 * LINE_FOLLOWING_SEARCH_MAX samples, brake and set b_lost.
 * Call under gh_mutex while following.
 *
 * @param[in] p_motion Motion status, for the heading.
 */
static void line_following_search(const motion_control_status_t * p_motion)
{
    int32_t side = (DRV_LINE_RIGHT == g_line.last_side) ? -1 : 1; /* +left. */
    /* Degrees turned anticlockwise since the search began, -180..179. */
    int32_t turned =
        line_following_off(p_motion->heading_deg, g_search.start_deg);

    turned *= side; /* Now towards last_side. */
    if ((1u == g_search.phase) && (turned >= LINE_FOLLOWING_SEARCH_DEG))
    {
        g_search.phase = 2u;
    }
    g_search.samples++; /* The search ends at SEARCH_MAX: fits. */
    if (((2u == g_search.phase) && (turned <= -LINE_FOLLOWING_SEARCH_DEG)) ||
        (g_search.samples >= LINE_FOLLOWING_SEARCH_MAX))
    {
        g_line.b_following = false;
        g_line.b_lost = true;
        motion_control_stop();
    }
    else
    {
        side = (1u == g_search.phase) ? side : -side;
        (void)motion_control_drive(-side * LINE_FOLLOWING_PIVOT_MM_S,
                                   side * LINE_FOLLOWING_PIVOT_MM_S);
    }
}

/*!
 * @brief Track sensor hits; hold a heading on a wobble, and stop on a
 * curve. Call under gh_mutex while following.
 *
 * @param[in] bits     Sensors over black now.
 * @param[in] prev     And at the last sample.
 * @param[in] p_motion Motion status, for the odometer and heading.
 * @return True while holding.
 */
static bool line_following_wobble(uint32_t bits, uint32_t prev,
                                  const motion_control_status_t * p_motion)
{
    bool const b_one = (DRV_LINE_LEFT == bits) || (DRV_LINE_RIGHT == bits);
    uint32_t const since_mm =
        motion_control_mm_since(g_wobble.hit_mm, p_motion->distance_mm);
    /* Degrees from the heading to the held one, +left. */
    int32_t const off =
        line_following_off(g_wobble.hold_deg, p_motion->heading_deg);
    uint32_t mid_deg = 0u;
    int32_t bend = 0;

    if (b_one && (bits != prev))
    {
        /* A new hit: the other side from the last, and close to it unless
         * holding, when hits come further apart. */
        g_wobble.hits =
            ((bits != g_wobble.side) &&
             ((g_wobble.b_hold) || (since_mm <= LINE_FOLLOWING_WOBBLE_MM)))
                ? ((g_wobble.hits < LINE_FOLLOWING_WOBBLE_HITS)
                       ? (g_wobble.hits + 1u)
                       : g_wobble.hits)
                : 1u;
        g_wobble.side = bits;
        g_wobble.hit_mm = p_motion->distance_mm;
        if (DRV_LINE_LEFT == bits)
        {
            g_wobble.left_deg = p_motion->heading_deg;
        }
        else
        {
            g_wobble.right_deg = p_motion->heading_deg;
        }
        /* Halfway round the shorter arc from right to left: 0..359.
         * (int32_t): right_deg is 0..359. */
        mid_deg = (uint32_t)(((int32_t)g_wobble.right_deg + 360 +
                              (line_following_off(g_wobble.left_deg,
                                                  g_wobble.right_deg) /
                               2)) %
                             360);
        bend = line_following_off(mid_deg, g_wobble.hold_deg);
        if ((g_wobble.b_hold) && ((bend > LINE_FOLLOWING_HOLD_BEND_DEG) ||
                                  (bend < -LINE_FOLLOWING_HOLD_BEND_DEG)))
        {
            /* The weave has turned: a curve. Start counting again. */
            g_wobble.hits = 1u;
        }
        /* Fewer hits: one side twice (a curve), or far apart. */
        g_wobble.b_hold = (LINE_FOLLOWING_WOBBLE_HITS == g_wobble.hits);
        g_wobble.hold_deg = mid_deg;
    }
    else if ((g_wobble.b_hold) && ((off > LINE_FOLLOWING_HOLD_OFF_DEG) ||
                                   (off < -LINE_FOLLOWING_HOLD_OFF_DEG)))
    {
        /* Steered well off the held heading: a curve. */
        g_wobble.b_hold = false;
        g_wobble.hits = 0u;
    }
    else if (b_one && (since_mm > LINE_FOLLOWING_HOLD_EXIT_MM))
    {
        /* A long hit: a curve or a real drift, so steer again. */
        g_wobble.b_hold = false;
        g_wobble.hits = 0u;
    }
    else
    {
        /* Same hit, or no hit. */
    }
    return g_wobble.b_hold;
}

/*!
 * @brief Degrees from from_deg round to to_deg, +anticlockwise (left),
 * -180..179. Both are headings, 0..359, so the casts keep them.
 */
static int32_t line_following_off(uint32_t to_deg, uint32_t from_deg)
{
    return ((((int32_t)to_deg - (int32_t)from_deg) + 540) % 360) - 180;
}

/*!
 * @brief Turn towards to_deg, gently. Call under gh_mutex while following.
 *
 * @param[in] p_motion Motion status, for the odometer and heading.
 * @param[in] to_deg   Heading to hold, 0..359.
 * @param[in] speed    Forward speed, mm/s.
 */
static void line_following_hold(const motion_control_status_t * p_motion,
                                uint32_t to_deg, int32_t speed)
{
    int32_t turn = line_following_off(to_deg, p_motion->heading_deg) *
                   LINE_FOLLOWING_HOLD_GAIN;

    turn = (turn > LINE_FOLLOWING_TURN_MIN) ? LINE_FOLLOWING_TURN_MIN : turn;
    turn = (turn < -LINE_FOLLOWING_TURN_MIN) ? -LINE_FOLLOWING_TURN_MIN : turn;
    g_steer.turn = turn; /* Within +-TURN_MIN: fits. */
    g_steer.from_mm = p_motion->distance_mm;
    turn = (speed * turn) / 1000; /* Wheel speed change, +left. */
    (void)motion_control_drive(speed - turn, speed + turn);
}

/*** end of file ***/
