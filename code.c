#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <pthread.h>
#include <sched.h>
#include <time.h>
#include <sys/neutrino.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <fcntl.h>
#include <sys/mman.h>

/*
 * ============================================================
 * QNX RTOS SMART ADAPTIVE TRAFFIC SIGNAL
 * Raspberry Pi 4 - BCM2711
 *
 * TWO-LANE SYSTEM
 *
 * Lane A:
 *      RED    GPIO17
 *      YELLOW GPIO27
 *      GREEN  GPIO22
 *
 * Lane B:
 *      RED    GPIO5
 *      YELLOW GPIO6
 *      GREEN  GPIO13
 *
 * Sensors:
 *      IR1       GPIO23 -> Lane A presence
 *      IR2       GPIO24 -> Lane B presence
 *      ULTRASONIC
 *      TRIG      GPIO25
 *      ECHO      GPIO12
 *
 * Architecture:
 *      Sensor Task
 *           |
 *      Sensor Fusion
 *           |
 *      Traffic Controller
 *           |
 *      Two-lane FSM
 *
 *      Priority Task
 *           |
 *      Priority Mode
 *
 * GPIO implementation:
 *      BCM2711 memory-mapped GPIO
 *
 * NOTE:
 * This version removes only the dependency on
 * <hw/gpio_api.h>. The traffic-control architecture,
 * pin assignments, sensors, timing, scheduling and
 * dashboard are retained.
 * ============================================================
 */


/* ============================================================
 * BCM2711 GPIO
 * ============================================================ */

#define GPIO_BASE_PHYS       0xFE200000ULL
#define GPIO_MAP_SIZE        0x1000

#define GPSET0               0x1C
#define GPCLR0               0x28
#define GPLEV0               0x34


/* ============================================================
 * GPIO DEFINITIONS
 * ============================================================ */

/* -------- Lane A -------- */
#define LANE_A_RED           17
#define LANE_A_YELLOW        27
#define LANE_A_GREEN         22

/* -------- Lane B -------- */
#define LANE_B_RED           5
#define LANE_B_YELLOW        6
#define LANE_B_GREEN         13

/* -------- Sensors -------- */
#define IR1_GPIO             23
#define IR2_GPIO             24
#define ULTRA_TRIG_GPIO      25
#define ULTRA_ECHO_GPIO      12


/* ============================================================
 * TIMING PARAMETERS
 * ============================================================ */

/* NORMAL MODE */
#define NORMAL_GREEN_MIN       8
#define NORMAL_GREEN_MAX      15
#define NORMAL_YELLOW_TIME     2
#define NORMAL_ALL_RED_TIME    1

/* PEAK MODE */
#define PEAK_GREEN_MIN        10
#define PEAK_GREEN_MAX        20
#define PEAK_YELLOW_TIME       2
#define PEAK_ALL_RED_TIME      1

/* PRIORITY MODE */
#define PRIORITY_YELLOW_TIME   2
#define PRIORITY_ALL_RED_TIME  1

/* Sensor update period */
#define SENSOR_PERIOD_MS     200

/* Ultrasonic configuration */
#define ULTRASONIC_TIMEOUT_US 30000
#define QUEUE_NEAR_CM         50
#define QUEUE_MEDIUM_CM       100
#define QUEUE_FAR_CM          200

/*
 * Persistent high-demand condition.
 * This is NOT ambulance identification.
 * It is a configured priority/clearance condition
 * for the prototype.
 */
#define PRIORITY_TIMEOUT_SEC  600


/* ============================================================
 * THREAD PRIORITIES
 * ============================================================ */

#define SENSOR_PRIORITY       20
#define TIMER_PRIORITY        15
#define CONTROLLER_PRIORITY   30
#define PRIORITY_TASK         40
#define DASHBOARD_PRIORITY    10


/* ============================================================
 * TRAFFIC MODES
 * ============================================================ */

typedef enum
{
    MODE_NORMAL = 0,
    MODE_PEAK,
    MODE_PRIORITY
} TrafficMode;


/* ============================================================
 * ACTIVE LANE
 * ============================================================ */

typedef enum
{
    LANE_A = 0,
    LANE_B
} ActiveLane;


/* ============================================================
 * SHARED SENSOR DATA
 * ============================================================ */

typedef struct
{
    int ir1;
    int ir2;
    double distance_cm;
    int ultrasonic_valid;
    int lane_a_demand;
    int lane_b_demand;
    int lane_a_score;
    int lane_b_score;
} SensorData;


/* ============================================================
 * SHARED SYSTEM STATE
 * ============================================================ */

typedef struct
{
    TrafficMode mode;
    ActiveLane active_lane;
    int priority_active;
    int congestion_timer_active;
    struct timespec congestion_start;
    SensorData sensors;
} TrafficState;


/* ============================================================
 * GLOBAL STATE
 * ============================================================ */

static TrafficState system_state;

static pthread_mutex_t state_mutex =
    PTHREAD_MUTEX_INITIALIZER;


/* ============================================================
 * DASHBOARD NETWORK BRIDGE
 * QNX Raspberry Pi = 10.0.0.1
 * Laptop dashboard  = 10.0.0.2
 * QNX listens on TCP port 9000.
 * ============================================================ */

#define DASHBOARD_BRIDGE_PORT  9000
#define DASHBOARD_MAX_CLIENTS  4
#define DASHBOARD_INTERVAL_MS  400

static int dashboard_clients[DASHBOARD_MAX_CLIENTS];
static int dashboard_client_count = 0;

static pthread_mutex_t dashboard_mutex =
    PTHREAD_MUTEX_INITIALIZER;

/* 0 = RED/ALL RED, 1 = YELLOW, 2 = GREEN */
static volatile int current_signal = 0;


/* ============================================================
 * BCM2711 GPIO REGISTER POINTER
 * ============================================================ */

static volatile uint32_t *gpio_regs = NULL;


/* ============================================================
 * DASHBOARD MODE STRING
 * ============================================================ */

static const char *dashboard_mode_string(TrafficMode mode)
{
    switch (mode)
    {
        case MODE_NORMAL:
            return "NORMAL";

        case MODE_PEAK:
            return "PEAK";

        case MODE_PRIORITY:
            return "PRIORITY";

        default:
            return "UNKNOWN";
    }
}


/* ============================================================
 * DASHBOARD SIGNAL STRING
 * ============================================================ */

static const char *dashboard_signal_string(int signal)
{
    switch (signal)
    {
        case 0:
            return "RED";

        case 1:
            return "YELLOW";

        case 2:
            return "GREEN";

        default:
            return "OFF";
    }
}


/* ============================================================
 * BCM2711 GPIO INITIALIZATION
 * ============================================================ */

static int initialize_gpio_memory(void)
{
    gpio_regs = (volatile uint32_t *)mmap_device_memory(
        NULL,
        GPIO_MAP_SIZE,
        PROT_READ | PROT_WRITE | PROT_NOCACHE,
        0,
        GPIO_BASE_PHYS
    );

    if (gpio_regs == MAP_FAILED)
    {
        gpio_regs = NULL;

        fprintf(
            stderr,
            "ERROR: Cannot map BCM2711 GPIO registers: %s\n",
            strerror(errno)
        );

        return -1;
    }

    return 0;
}


/* ============================================================
 * GPIO FUNCTION SELECT
 *
 * 000 = INPUT
 * 001 = OUTPUT
 * ============================================================ */

static int gpio_set_function(int pin, int output)
{
    volatile uint32_t *reg;
    uint32_t value;
    int register_index;
    int shift;

    if (gpio_regs == NULL)
        return -1;

    if (pin < 0 || pin > 53)
        return -1;

    register_index = pin / 10;
    shift = (pin % 10) * 3;

    reg = gpio_regs + register_index;

    value = *reg;

    value &= ~(7U << shift);

    if (output)
    {
        value |= (1U << shift);
    }

    *reg = value;

    return 0;
}


/* ============================================================
 * GPIO INPUT
 * ============================================================ */

static int gpio_input(int pin)
{
    return gpio_set_function(pin, 0);
}


/* ============================================================
 * GPIO WRITE
 * ============================================================ */

static int gpio_write(int pin, int value)
{
    if (gpio_regs == NULL)
        return -1;

    if (pin < 0 || pin > 31)
        return -1;

    if (value)
    {
        gpio_regs[GPSET0 / 4] =
            (1U << pin);
    }
    else
    {
        gpio_regs[GPCLR0 / 4] =
            (1U << pin);
    }

    return 0;
}


/* ============================================================
 * GPIO READ
 * ============================================================ */

static int gpio_read(int pin)
{
    uint32_t value;

    if (gpio_regs == NULL)
        return -1;

    if (pin < 0 || pin > 31)
        return -1;

    value = gpio_regs[GPLEV0 / 4];

    if (value & (1U << pin))
        return 1;

    return 0;
}


/* ============================================================
 * TRAFFIC LIGHT CONTROL
 * ============================================================ */

static void lane_a_all_off(void)
{
    gpio_write(LANE_A_RED, 0);
    gpio_write(LANE_A_YELLOW, 0);
    gpio_write(LANE_A_GREEN, 0);
}

static void lane_b_all_off(void)
{
    gpio_write(LANE_B_RED, 0);
    gpio_write(LANE_B_YELLOW, 0);
    gpio_write(LANE_B_GREEN, 0);
}

static void all_lights_off(void)
{
    lane_a_all_off();
    lane_b_all_off();
}


/* ============================================================
 * ALL RED
 * ============================================================ */

static void all_red(void)
{
    all_lights_off();

    gpio_write(LANE_A_RED, 1);
    gpio_write(LANE_B_RED, 1);

    current_signal = 0;

    printf("[SIGNAL] ALL RED\n");
    fflush(stdout);
}


/* ============================================================
 * LANE A GREEN
 * ============================================================ */

static void lane_a_green(void)
{
    all_lights_off();

    gpio_write(LANE_B_RED, 1);
    gpio_write(LANE_A_GREEN, 1);

    current_signal = 2;

    printf(
        "[SIGNAL] LANE A = GREEN | LANE B = RED\n"
    );

    fflush(stdout);
}


/* ============================================================
 * LANE A YELLOW
 * ============================================================ */

static void lane_a_yellow(void)
{
    all_lights_off();

    gpio_write(LANE_B_RED, 1);
    gpio_write(LANE_A_YELLOW, 1);

    current_signal = 1;

    printf(
        "[SIGNAL] LANE A = YELLOW | LANE B = RED\n"
    );

    fflush(stdout);
}


/* ============================================================
 * LANE B GREEN
 * ============================================================ */

static void lane_b_green(void)
{
    all_lights_off();

    gpio_write(LANE_A_RED, 1);
    gpio_write(LANE_B_GREEN, 1);

    current_signal = 2;

    printf(
        "[SIGNAL] LANE A = RED | LANE B = GREEN\n"
    );

    fflush(stdout);
}


/* ============================================================
 * LANE B YELLOW
 * ============================================================ */

static void lane_b_yellow(void)
{
    all_lights_off();

    gpio_write(LANE_A_RED, 1);
    gpio_write(LANE_B_YELLOW, 1);

    current_signal = 1;

    printf(
        "[SIGNAL] LANE A = RED | LANE B = YELLOW\n"
    );

    fflush(stdout);
}


/* ============================================================
 * VEHICLE DETECTION
 *
 * LOW  = vehicle detected
 * HIGH = no vehicle
 * ============================================================ */

static int ir_vehicle_detected(int pin)
{
    int raw;

    raw = gpio_read(pin);

    if (raw < 0)
    {
        return 0;
    }

    return raw == 0;
}


/* ============================================================
 * MICROSECOND TIME
 * ============================================================ */

static uint64_t get_time_us(void)
{
    struct timespec ts;

    clock_gettime(
        CLOCK_MONOTONIC,
        &ts
    );

    return
        ((uint64_t)ts.tv_sec * 1000000ULL) +
        ((uint64_t)ts.tv_nsec / 1000ULL);
}


/* ============================================================
 * ULTRASONIC SENSOR
 *
 * HC-SR04-type operation:
 * 1. TRIG HIGH for >=10 us
 * 2. TRIG LOW
 * 3. Wait for ECHO HIGH
 * 4. Measure ECHO HIGH duration
 * 5. distance = time / 58
 * ============================================================ */

static double ultrasonic_read_cm(void)
{
    uint64_t start;
    uint64_t echo_start;
    uint64_t echo_end;
    int echo;

    gpio_write(
        ULTRA_TRIG_GPIO,
        0
    );

    usleep(2);

    gpio_write(
        ULTRA_TRIG_GPIO,
        1
    );

    usleep(10);

    gpio_write(
        ULTRA_TRIG_GPIO,
        0
    );

    start = get_time_us();

    while (gpio_read(ULTRA_ECHO_GPIO) == 0)
    {
        if ((get_time_us() - start) >
            ULTRASONIC_TIMEOUT_US)
        {
            return -1.0;
        }
    }

    echo_start = get_time_us();

    while (1)
    {
        echo = gpio_read(ULTRA_ECHO_GPIO);

        if (echo == 0)
        {
            break;
        }

        if ((get_time_us() - echo_start) >
            ULTRASONIC_TIMEOUT_US)
        {
            return -1.0;
        }
    }

    echo_end = get_time_us();

    return
        (double)(echo_end - echo_start) /
        58.0;
}


/* ============================================================
 * SENSOR SCORE
 * ============================================================ */

static int calculate_lane_score(
    int ir_detected,
    double distance_cm,
    int ultrasonic_valid)
{
    int score = 0;

    if (ir_detected)
    {
        score += 3;
    }

    if (ultrasonic_valid)
    {
        if (distance_cm <= QUEUE_NEAR_CM)
        {
            score += 5;
        }
        else if (distance_cm <= QUEUE_MEDIUM_CM)
        {
            score += 3;
        }
        else if (distance_cm <= QUEUE_FAR_CM)
        {
            score += 1;
        }
    }

    return score;
}


/* ============================================================
 * SENSOR FUSION
 * ============================================================ */

static void update_sensor_state(
    int ir1,
    int ir2,
    double distance_cm)
{
    pthread_mutex_lock(&state_mutex);

    system_state.sensors.ir1 = ir1;
    system_state.sensors.ir2 = ir2;

    system_state.sensors.distance_cm =
        distance_cm;

    system_state.sensors.ultrasonic_valid =
        distance_cm > 0.0;

    system_state.sensors.lane_a_demand =
        ir1;

    system_state.sensors.lane_b_demand =
        ir2;

    /*
     * One ultrasonic sensor represents the
     * monitored intersection queue.
     *
     * When both approaches are occupied,
     * the distance contributes to both lane
     * scores.
     */
    if (ir1 && ir2)
    {
        system_state.sensors.lane_a_score =
            calculate_lane_score(
                ir1,
                distance_cm,
                distance_cm > 0.0
            );

        system_state.sensors.lane_b_score =
            calculate_lane_score(
                ir2,
                distance_cm,
                distance_cm > 0.0
            );
    }
    else
    {
        system_state.sensors.lane_a_score =
            calculate_lane_score(
                ir1,
                -1.0,
                0
            );

        system_state.sensors.lane_b_score =
            calculate_lane_score(
                ir2,
                -1.0,
                0
            );
    }

    pthread_mutex_unlock(&state_mutex);
}


/* ============================================================
 * SELECT LANE
 * ============================================================ */

static ActiveLane select_lane(void)
{
    ActiveLane lane;
    int score_a;
    int score_b;

    pthread_mutex_lock(&state_mutex);

    score_a =
        system_state.sensors.lane_a_score;

    score_b =
        system_state.sensors.lane_b_score;

    lane =
        system_state.active_lane;

    pthread_mutex_unlock(&state_mutex);

    if (score_a > 0 && score_b == 0)
    {
        return LANE_A;
    }

    if (score_b > 0 && score_a == 0)
    {
        return LANE_B;
    }

    if (score_a > score_b)
    {
        return LANE_A;
    }

    if (score_b > score_a)
    {
        return LANE_B;
    }

    if (lane == LANE_A)
    {
        return LANE_B;
    }

    return LANE_A;
}


/* ============================================================
 * GET MODE
 * ============================================================ */

static TrafficMode get_mode(void)
{
    TrafficMode mode;

    pthread_mutex_lock(&state_mutex);

    mode = system_state.mode;

    pthread_mutex_unlock(&state_mutex);

    return mode;
}


/* ============================================================
 * SET MODE
 * ============================================================ */

static void set_mode(TrafficMode mode)
{
    pthread_mutex_lock(&state_mutex);

    system_state.mode = mode;

    pthread_mutex_unlock(&state_mutex);
}


/* ============================================================
 * INTERRUPTIBLE DELAY
 * ============================================================ */

static int traffic_delay(
    int seconds,
    TrafficMode expected_mode)
{
    int i;

    for (i = 0; i < seconds * 10; i++)
    {
        usleep(100000);

        if (get_mode() != expected_mode)
        {
            return 0;
        }
    }

    return 1;
}


/* ============================================================
 * SENSOR TASK
 * ============================================================ */

static void *sensor_task(void *arg)
{
    (void)arg;

    while (1)
    {
        int ir1;
        int ir2;
        double distance;

        ir1 =
            ir_vehicle_detected(IR1_GPIO);

        ir2 =
            ir_vehicle_detected(IR2_GPIO);

        distance =
            ultrasonic_read_cm();

        update_sensor_state(
            ir1,
            ir2,
            distance
        );

        printf(
            "\n"
            "[SENSOR]\n"
            "  IR1 Lane A       : %s\n"
            "  IR2 Lane B       : %s\n",
            ir1 ? "VEHICLE" : "CLEAR",
            ir2 ? "VEHICLE" : "CLEAR"
        );

        if (distance > 0.0)
        {
            printf(
                "  Ultrasonic       : %.1f cm\n",
                distance
            );
        }
        else
        {
            printf(
                "  Ultrasonic       : TIMEOUT\n"
            );
        }

        pthread_mutex_lock(&state_mutex);

        printf(
            "  Lane A Score     : %d\n"
            "  Lane B Score     : %d\n",
            system_state.sensors.lane_a_score,
            system_state.sensors.lane_b_score
        );

        pthread_mutex_unlock(&state_mutex);

        pthread_mutex_lock(&state_mutex);

        if (!system_state.priority_active)
        {
            if (ir1 && ir2)
            {
                system_state.mode =
                    MODE_PEAK;

                if (!system_state.congestion_timer_active)
                {
                    clock_gettime(
                        CLOCK_MONOTONIC,
                        &system_state.congestion_start
                    );

                    system_state.congestion_timer_active =
                        1;

                    printf(
                        "\n"
                        "[SCHEDULER] HIGH DEMAND\n"
                        "[SCHEDULER] CONGESTION TIMER STARTED\n"
                    );
                }
            }
            else
            {
                system_state.mode =
                    MODE_NORMAL;

                system_state.congestion_timer_active =
                    0;
            }
        }

        pthread_mutex_unlock(&state_mutex);

        usleep(SENSOR_PERIOD_MS * 1000);
    }

    return NULL;
}


/* ============================================================
 * PRIORITY TASK
 * ============================================================ */

static void *priority_task(void *arg)
{
    (void)arg;

    while (1)
    {
        pthread_mutex_lock(&state_mutex);

        if (!system_state.priority_active &&
            system_state.congestion_timer_active &&
            system_state.sensors.ir1 &&
            system_state.sensors.ir2)
        {
            struct timespec now;
            long elapsed;

            clock_gettime(
                CLOCK_MONOTONIC,
                &now
            );

            elapsed =
                now.tv_sec -
                system_state.congestion_start.tv_sec;

            if (elapsed >=
                PRIORITY_TIMEOUT_SEC)
            {
                system_state.priority_active = 1;

                system_state.mode =
                    MODE_PRIORITY;

                printf(
                    "\n"
                    "============================================\n"
                    " PRIORITY / CLEARANCE MODE ACTIVATED\n"
                    " Persistent high-demand condition detected\n"
                    "============================================\n"
                );
            }
        }

        if (system_state.priority_active &&
            !system_state.sensors.ir1 &&
            !system_state.sensors.ir2)
        {
            system_state.priority_active = 0;

            system_state.congestion_timer_active =
                0;

            system_state.mode =
                MODE_NORMAL;

            printf(
                "\n"
                "============================================\n"
                " PRIORITY CONDITION CLEARED\n"
                " RETURNING TO NORMAL SCHEDULING\n"
                "============================================\n"
            );
        }

        pthread_mutex_unlock(&state_mutex);

        usleep(100000);
    }

    return NULL;
}


/* ============================================================
 * TIMER / TELEMETRY TASK
 * ============================================================ */

static void *timer_task(void *arg)
{
    (void)arg;

    while (1)
    {
        pthread_mutex_lock(&state_mutex);

        if (system_state.congestion_timer_active)
        {
            struct timespec now;
            long elapsed;

            clock_gettime(
                CLOCK_MONOTONIC,
                &now
            );

            elapsed =
                now.tv_sec -
                system_state.congestion_start.tv_sec;

            printf(
                "[TIMER] High-demand duration: %ld sec\n",
                elapsed
            );
        }

        pthread_mutex_unlock(&state_mutex);

        sleep(1);
    }

    return NULL;
}


/* ============================================================
 * NORMAL LANE CYCLE
 * ============================================================ */

static void run_normal_lane(ActiveLane lane)
{
    all_red();

    sleep(NORMAL_ALL_RED_TIME);

    if (lane == LANE_A)
    {
        lane_a_green();

        if (!traffic_delay(
                NORMAL_GREEN_MAX,
                MODE_NORMAL))
        {
            return;
        }

        lane_a_yellow();

        traffic_delay(
            NORMAL_YELLOW_TIME,
            MODE_NORMAL
        );

        all_red();
    }
    else
    {
        lane_b_green();

        if (!traffic_delay(
                NORMAL_GREEN_MAX,
                MODE_NORMAL))
        {
            return;
        }

        lane_b_yellow();

        traffic_delay(
            NORMAL_YELLOW_TIME,
            MODE_NORMAL
        );

        all_red();
    }
}


/* ============================================================
 * PEAK LANE CYCLE
 * ============================================================ */

static void run_peak_lane(ActiveLane lane)
{
    all_red();

    sleep(PEAK_ALL_RED_TIME);

    if (lane == LANE_A)
    {
        printf(
            "[ADAPTIVE] Lane A receives extended green\n"
        );

        lane_a_green();

        traffic_delay(
            PEAK_GREEN_MAX,
            MODE_PEAK
        );

        lane_a_yellow();

        traffic_delay(
            PEAK_YELLOW_TIME,
            MODE_PEAK
        );
    }
    else
    {
        printf(
            "[ADAPTIVE] Lane B receives extended green\n"
        );

        lane_b_green();

        traffic_delay(
            PEAK_GREEN_MAX,
            MODE_PEAK
        );

        lane_b_yellow();

        traffic_delay(
            PEAK_YELLOW_TIME,
            MODE_PEAK
        );
    }

    all_red();
}


/* ============================================================
 * PRIORITY CYCLE
 * ============================================================ */

static void run_priority(void)
{
    ActiveLane priority_lane;

    priority_lane = select_lane();

    all_red();

    sleep(PRIORITY_ALL_RED_TIME);

    printf(
        "\n"
        "============================================\n"
        " PRIORITY SCHEDULER\n"
        " Selected lane: %s\n"
        "============================================\n",
        priority_lane == LANE_A ?
        "LANE A" :
        "LANE B"
    );

    if (priority_lane == LANE_A)
    {
        lane_a_green();
    }
    else
    {
        lane_b_green();
    }

    while (get_mode() == MODE_PRIORITY)
    {
        usleep(100000);
    }

    all_red();
}


/* ============================================================
 * TRAFFIC CONTROLLER
 * ============================================================ */

static void *controller_task(void *arg)
{
    TrafficMode previous_mode = MODE_NORMAL;

    (void)arg;

    while (1)
    {
        TrafficMode mode;
        ActiveLane selected_lane;

        mode = get_mode();

        if (mode != previous_mode)
        {
            printf(
                "\n"
                "============================================\n"
            );

            if (mode == MODE_NORMAL)
            {
                printf(
                    " TRAFFIC MODE : NORMAL\n"
                );
            }
            else if (mode == MODE_PEAK)
            {
                printf(
                    " TRAFFIC MODE : PEAK / ADAPTIVE\n"
                );
            }
            else
            {
                printf(
                    " TRAFFIC MODE : PRIORITY / CLEARANCE\n"
                );
            }

            printf(
                "============================================\n"
            );

            previous_mode = mode;
        }

        if (mode == MODE_NORMAL)
        {
            selected_lane = select_lane();

            pthread_mutex_lock(&state_mutex);

            system_state.active_lane =
                selected_lane;

            pthread_mutex_unlock(&state_mutex);

            run_normal_lane(selected_lane);
        }
        else if (mode == MODE_PEAK)
        {
            selected_lane = select_lane();

            pthread_mutex_lock(&state_mutex);

            system_state.active_lane =
                selected_lane;

            pthread_mutex_unlock(&state_mutex);

            run_peak_lane(selected_lane);
        }
        else
        {
            run_priority();
        }

        previous_mode = mode;
    }

    return NULL;
}


/* ============================================================
 * DASHBOARD CLIENT REMOVE
 * ============================================================ */

static void dashboard_remove_client(int index)
{
    close(dashboard_clients[index]);

    if (index < dashboard_client_count - 1)
    {
        memmove(
            &dashboard_clients[index],
            &dashboard_clients[index + 1],
            (dashboard_client_count - index - 1) *
            sizeof(dashboard_clients[0])
        );
    }

    dashboard_client_count--;
}


/* ============================================================
 * DASHBOARD BROADCAST
 * ============================================================ */

static void dashboard_broadcast(const char *message)
{
    int i = 0;

    pthread_mutex_lock(&dashboard_mutex);

    while (i < dashboard_client_count)
    {
        ssize_t sent;

        sent = send(
            dashboard_clients[i],
            message,
            strlen(message),
            MSG_NOSIGNAL
        );

        if (sent <= 0)
        {
            printf(
                "[DASHBOARD] Client disconnected\n"
            );

            dashboard_remove_client(i);
            continue;
        }

        i++;
    }

    pthread_mutex_unlock(&dashboard_mutex);
}


/* ============================================================
 * DASHBOARD STATUS
 * ============================================================ */

static void dashboard_send_status(void)
{
    SensorData sensors;
    TrafficMode mode;
    ActiveLane active_lane;
    int priority;
    int congestion;
    int signal;

    struct timespec ts;
    char json[1024];

    pthread_mutex_lock(&state_mutex);

    sensors = system_state.sensors;
    mode = system_state.mode;
    active_lane = system_state.active_lane;
    priority = system_state.priority_active;
    congestion = system_state.congestion_timer_active;

    pthread_mutex_unlock(&state_mutex);

    signal = current_signal;

    clock_gettime(
        CLOCK_REALTIME,
        &ts
    );

    snprintf(
        json,
        sizeof(json),
        "{"
        "\"type\":\"traffic_status\","
        "\"mode\":%d,"
        "\"mode_str\":\"%s\","
        "\"active_lane\":%d,"
        "\"active_lane_str\":\"%s\","
        "\"priority\":%d,"
        "\"congestion\":%d,"
        "\"ir1\":%d,"
        "\"ir2\":%d,"
        "\"vehicle1\":%d,"
        "\"vehicle2\":%d,"
        "\"distance_cm\":%.1f,"
        "\"ultrasonic_valid\":%d,"
        "\"lane_a_score\":%d,"
        "\"lane_b_score\":%d,"
        "\"signal\":%d,"
        "\"signal_str\":\"%s\","
        "\"timestamp_sec\":%ld,"
        "\"timestamp_nsec\":%ld"
        "}\n",
        (int)mode,
        dashboard_mode_string(mode),
        (int)active_lane,
        active_lane == LANE_A ?
            "LANE_A" : "LANE_B",
        priority,
        congestion,
        sensors.ir1,
        sensors.ir2,
        sensors.lane_a_demand,
        sensors.lane_b_demand,
        sensors.distance_cm,
        sensors.ultrasonic_valid,
        sensors.lane_a_score,
        sensors.lane_b_score,
        signal,
        dashboard_signal_string(signal),
        (long)ts.tv_sec,
        (long)ts.tv_nsec
    );

    dashboard_broadcast(json);
}


/* ============================================================
 * DASHBOARD NETWORK THREAD
 * ============================================================ */

static void *dashboard_network_thread(void *arg)
{
    int server_fd;
    int client_fd;
    int option = 1;
    int flags;

    struct sockaddr_in server_addr;
    struct sockaddr_in client_addr;

    socklen_t client_len;

    struct timespec delay;

    (void)arg;

    {
        struct sched_param param;

        memset(
            &param,
            0,
            sizeof(param)
        );

        param.sched_priority =
            DASHBOARD_PRIORITY;

        pthread_setschedparam(
            pthread_self(),
            SCHED_FIFO,
            &param
        );
    }

    printf(
        "\n[DASHBOARD] Network thread started\n"
    );

    printf(
        "[DASHBOARD] QNX server: 10.0.0.1:%d\n",
        DASHBOARD_BRIDGE_PORT
    );

    printf(
        "[DASHBOARD] Laptop: 10.0.0.2:3000\n"
    );

    server_fd =
        socket(
            AF_INET,
            SOCK_STREAM,
            0
        );

    if (server_fd < 0)
    {
        printf(
            "[DASHBOARD] ERROR: socket() failed: %s\n",
            strerror(errno)
        );

        return NULL;
    }

    setsockopt(
        server_fd,
        SOL_SOCKET,
        SO_REUSEADDR,
        &option,
        sizeof(option)
    );

    memset(
        &server_addr,
        0,
        sizeof(server_addr)
    );

    server_addr.sin_family =
        AF_INET;

    server_addr.sin_addr.s_addr =
        htonl(INADDR_ANY);

    server_addr.sin_port =
        htons(DASHBOARD_BRIDGE_PORT);

    if (bind(
            server_fd,
            (struct sockaddr *)&server_addr,
            sizeof(server_addr)
        ) < 0)
    {
        printf(
            "[DASHBOARD] ERROR: bind port %d failed: %s\n",
            DASHBOARD_BRIDGE_PORT,
            strerror(errno)
        );

        close(server_fd);

        return NULL;
    }

    if (listen(
            server_fd,
            DASHBOARD_MAX_CLIENTS
        ) < 0)
    {
        printf(
            "[DASHBOARD] ERROR: listen() failed: %s\n",
            strerror(errno)
        );

        close(server_fd);

        return NULL;
    }

    flags =
        fcntl(
            server_fd,
            F_GETFL,
            0
        );

    if (flags >= 0)
    {
        fcntl(
            server_fd,
            F_SETFL,
            flags | O_NONBLOCK
        );
    }

    printf(
        "[DASHBOARD] TCP server ready on port %d\n",
        DASHBOARD_BRIDGE_PORT
    );

    delay.tv_sec = 0;

    delay.tv_nsec =
        DASHBOARD_INTERVAL_MS *
        1000000L;

    while (1)
    {
        client_len =
            sizeof(client_addr);

        client_fd =
            accept(
                server_fd,
                (struct sockaddr *)&client_addr,
                &client_len
            );

        if (client_fd >= 0)
        {
            pthread_mutex_lock(
                &dashboard_mutex
            );

            if (dashboard_client_count <
                DASHBOARD_MAX_CLIENTS)
            {
                dashboard_clients[
                    dashboard_client_count] =
                    client_fd;

                dashboard_client_count++;

                printf(
                    "[DASHBOARD] Laptop connected from %s "
                    "(clients=%d)\n",
                    inet_ntoa(
                        client_addr.sin_addr
                    ),
                    dashboard_client_count
                );
            }
            else
            {
                printf(
                    "[DASHBOARD] Maximum clients reached\n"
                );

                close(client_fd);
            }

            pthread_mutex_unlock(
                &dashboard_mutex
            );
        }

        pthread_mutex_lock(
            &dashboard_mutex
        );

        if (dashboard_client_count > 0)
        {
            pthread_mutex_unlock(
                &dashboard_mutex
            );

            dashboard_send_status();
        }
        else
        {
            pthread_mutex_unlock(
                &dashboard_mutex
            );
        }

        nanosleep(
            &delay,
            NULL
        );
    }

    close(server_fd);

    return NULL;
}


/* ============================================================
 * GPIO INITIALIZATION
 * ============================================================ */

static int initialize_gpio(void)
{
    if (initialize_gpio_memory() != 0)
    {
        fprintf(
            stderr,
            "GPIO memory initialization failed.\n"
        );

        return -1;
    }

    /* Six traffic LEDs */
    if (gpio_set_function(LANE_A_RED, 1) != 0)
        return -1;

    if (gpio_set_function(LANE_A_YELLOW, 1) != 0)
        return -1;

    if (gpio_set_function(LANE_A_GREEN, 1) != 0)
        return -1;

    if (gpio_set_function(LANE_B_RED, 1) != 0)
        return -1;

    if (gpio_set_function(LANE_B_YELLOW, 1) != 0)
        return -1;

    if (gpio_set_function(LANE_B_GREEN, 1) != 0)
        return -1;

    /* IR sensors */
    if (gpio_input(IR1_GPIO) != 0)
        return -1;

    if (gpio_input(IR2_GPIO) != 0)
        return -1;

    /* Ultrasonic trigger */
    if (gpio_set_function(
            ULTRA_TRIG_GPIO,
            1
        ) != 0)
        return -1;

    /* Ultrasonic echo */
    if (gpio_input(
            ULTRA_ECHO_GPIO
        ) != 0)
        return -1;

    /* Initial output states */
    gpio_write(LANE_A_RED, 0);
    gpio_write(LANE_A_YELLOW, 0);
    gpio_write(LANE_A_GREEN, 0);

    gpio_write(LANE_B_RED, 0);
    gpio_write(LANE_B_YELLOW, 0);
    gpio_write(LANE_B_GREEN, 0);

    gpio_write(ULTRA_TRIG_GPIO, 0);

    all_red();

    printf(
        "[GPIO] BCM2711 GPIO initialization complete\n"
    );

    return 0;
}


/* ============================================================
 * REAL-TIME THREAD CREATION
 * ============================================================ */

static int create_rt_thread(
    pthread_t *thread,
    void *(*function)(void *),
    int priority)
{
    pthread_attr_t attr;
    struct sched_param param;
    int rc;

    rc = pthread_attr_init(&attr);

    if (rc != 0)
        return rc;

    rc = pthread_attr_setschedpolicy(
        &attr,
        SCHED_FIFO
    );

    if (rc != 0)
    {
        pthread_attr_destroy(&attr);
        return rc;
    }

    rc = pthread_attr_setinheritsched(
        &attr,
        PTHREAD_EXPLICIT_SCHED
    );

    if (rc != 0)
    {
        pthread_attr_destroy(&attr);
        return rc;
    }

    memset(
        &param,
        0,
        sizeof(param)
    );

    param.sched_priority =
        priority;

    rc = pthread_attr_setschedparam(
        &attr,
        &param
    );

    if (rc != 0)
    {
        pthread_attr_destroy(&attr);
        return rc;
    }

    rc = pthread_create(
        thread,
        &attr,
        function,
        NULL
    );

    pthread_attr_destroy(&attr);

    return rc;
}


/* ============================================================
 * MAIN
 * ============================================================ */

int main(void)
{
    pthread_t sensor_thread;
    pthread_t controller_thread;
    pthread_t priority_thread;
    pthread_t timer_thread;
    pthread_t dashboard_thread;

    printf(
        "\n"
        "==================================================\n"
        " QNX RTOS SMART ADAPTIVE TRAFFIC SIGNAL\n"
        " Raspberry Pi 4 - BCM2711\n"
        " TWO-LANE SENSOR-FUSION ARCHITECTURE\n"
        "==================================================\n\n"
    );

    printf(
        "SYSTEM CONFIGURATION\n"
        "--------------------\n"
        "Lane A : GPIO17/27/22\n"
        "Lane B : GPIO5/6/13\n"
        "IR1   : GPIO23\n"
        "IR2   : GPIO24\n"
        "TRIG  : GPIO25\n"
        "ECHO  : GPIO12\n\n"
    );

    if (initialize_gpio() != 0)
    {
        fprintf(
            stderr,
            "GPIO initialization failed.\n"
        );

        return EXIT_FAILURE;
    }

    memset(
        &system_state,
        0,
        sizeof(system_state)
    );

    system_state.mode =
        MODE_NORMAL;

    system_state.active_lane =
        LANE_A;

    if (create_rt_thread(
            &sensor_thread,
            sensor_task,
            SENSOR_PRIORITY
        ) != 0)
    {
        fprintf(
            stderr,
            "Sensor thread creation failed\n"
        );

        return EXIT_FAILURE;
    }

    if (create_rt_thread(
            &priority_thread,
            priority_task,
            PRIORITY_TASK
        ) != 0)
    {
        fprintf(
            stderr,
            "Priority thread creation failed\n"
        );

        return EXIT_FAILURE;
    }

    if (create_rt_thread(
            &timer_thread,
            timer_task,
            TIMER_PRIORITY
        ) != 0)
    {
        fprintf(
            stderr,
            "Timer thread creation failed\n"
        );

        return EXIT_FAILURE;
    }

    if (create_rt_thread(
            &controller_thread,
            controller_task,
            CONTROLLER_PRIORITY
        ) != 0)
    {
        fprintf(
            stderr,
            "Controller thread creation failed\n"
        );

        return EXIT_FAILURE;
    }

    if (create_rt_thread(
            &dashboard_thread,
            dashboard_network_thread,
            DASHBOARD_PRIORITY
        ) != 0)
    {
        fprintf(
            stderr,
            "Dashboard network thread creation failed\n"
        );

        return EXIT_FAILURE;
    }

    pthread_join(
        sensor_thread,
        NULL
    );

    pthread_join(
        priority_thread,
        NULL
    );

    pthread_join(
        timer_thread,
        NULL
    );

    pthread_join(
        controller_thread,
        NULL
    );

    pthread_join(
        dashboard_thread,
        NULL
    );

    all_red();

    if (gpio_regs != NULL)
    {
    	munmap_device_memory(
    	    (void *)gpio_regs,
    	    GPIO_MAP_SIZE
    	);
        gpio_regs = NULL;
    }

    return EXIT_SUCCESS;
}