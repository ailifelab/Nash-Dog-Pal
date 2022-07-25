/**
 * @file joint_esp32_microros.ino
 * @author R Woodkite
 * @brief
 * @version 0.1
 * @date 2022-07-25
 *
 * @copyright Copyleft ailifelab.org
 *
 */

// Servo Library for ESP32 (https://github.com/madhephaestus/ESP32Servo)
#include <ESP32Servo.h>
#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <pthread.h>
#include <example_interfaces/action/fibonacci.h>

#include "MultiStepper.h"
#include "AccelStepper.h"

#define RCCHECK(fn)                  \
    {                                \
        rcl_ret_t temp_rc = fn;      \
        if ((temp_rc != RCL_RET_OK)) \
        {                            \
            while (1)                \
            {                        \
            };                       \
        }                            \
    }
#define RCSOFTCHECK(fn)              \
    {                                \
        rcl_ret_t temp_rc = fn;      \
        if ((temp_rc != RCL_RET_OK)) \
        {                            \
        }                            \
    }

rcl_node_t node;
rclc_support_t support;
rcl_allocator_t allocator;

rclc_action_server_t action_server;
rclc_executor_t executor;

rcl_service_t service;
rcl_wait_set_t wait_set;

example_interfaces__action__Fibonacci_SendGoal_Request ros_goal_request[10];

void *fibonacci_worker(void *args)
{
    (void)args;
    rclc_action_goal_handle_t *goal_handle = (rclc_action_goal_handle_t *)args;
    rcl_action_goal_state_t goal_state;

    example_interfaces__action__Fibonacci_SendGoal_Request *req =
        (example_interfaces__action__Fibonacci_SendGoal_Request *)goal_handle->ros_goal_request;

    example_interfaces__action__Fibonacci_GetResult_Response response = {0};
    example_interfaces__action__Fibonacci_FeedbackMessage feedback;

    if (req->goal.order < 2)
    {
        goal_state = GOAL_STATE_ABORTED;
    }
    else
    {
        feedback.feedback.sequence.capacity = req->goal.order;
        feedback.feedback.sequence.size = 0;
        feedback.feedback.sequence.data =
            (int32_t *)malloc(feedback.feedback.sequence.capacity * sizeof(int32_t));

        feedback.feedback.sequence.data[0] = 0;
        feedback.feedback.sequence.data[1] = 1;
        feedback.feedback.sequence.size = 2;

        for (size_t i = 2; i < (size_t)req->goal.order && !goal_handle->goal_cancelled; i++)
        {
            feedback.feedback.sequence.data[i] = feedback.feedback.sequence.data[i - 1] +
                                                 feedback.feedback.sequence.data[i - 2];
            feedback.feedback.sequence.size++;

            Serial.print("Publishing feedback\n");
            rclc_action_publish_feedback(goal_handle, &feedback);
            usleep(500000);
        }

        if (!goal_handle->goal_cancelled)
        {
            response.result.sequence.capacity = feedback.feedback.sequence.capacity;
            response.result.sequence.size = feedback.feedback.sequence.size;
            response.result.sequence.data = feedback.feedback.sequence.data;
            goal_state = GOAL_STATE_SUCCEEDED;
        }
        else
        {
            goal_state = GOAL_STATE_CANCELED;
        }
    }

    rcl_ret_t rc;
    do
    {
        rc = rclc_action_send_result(goal_handle, goal_state, &response);
        usleep(1e6);
    } while (rc != RCL_RET_OK);

    free(feedback.feedback.sequence.data);
    pthread_exit(NULL);
}

rcl_ret_t handle_goal(rclc_action_goal_handle_t *goal_handle, void *context)
{
    (void)context;

    example_interfaces__action__Fibonacci_SendGoal_Request *req =
        (example_interfaces__action__Fibonacci_SendGoal_Request *)goal_handle->ros_goal_request;

    // Too big, rejecting
    if (req->goal.order > 200)
    {
        return RCL_RET_ACTION_GOAL_REJECTED;
    }

    pthread_t *thread_id = (pthread_t*) malloc(sizeof(pthread_t));
    pthread_create(thread_id, NULL, fibonacci_worker, goal_handle);
    return RCL_RET_ACTION_GOAL_ACCEPTED;
}

bool handle_cancel(rclc_action_goal_handle_t *goal_handle, void *context)
{
    (void)context;
    (void)goal_handle;

    return true;
}

void setup()
{
    Serial.begin(115200, SERIAL_8N1);
    set_microros_transports();
    delay(1000);

    allocator = rcl_get_default_allocator();

    // create init_options
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

    // create node
    RCCHECK(rclc_node_init_default(&node, "fibonacci_action_server_rcl", "", &support));

    // Create action service
    RCCHECK(
        rclc_action_server_init_default(
            &action_server,
            &node,
            &support,
            ROSIDL_GET_ACTION_TYPE_SUPPORT(example_interfaces, Fibonacci),
            "fibonacci"));
    // create executor
    RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));

    RCCHECK(
        rclc_executor_add_action_server(
            &executor,
            &action_server,
            10,
            ros_goal_request,
            sizeof(example_interfaces__action__Fibonacci_SendGoal_Request),
            handle_goal,
            handle_cancel,
            (void *)&action_server));
}

void loop()
{
    delay(100);
    RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}
