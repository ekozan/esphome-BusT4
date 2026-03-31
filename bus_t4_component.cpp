// Updated bus_t4_component.cpp to fix queue initialization race condition

// Include necessary headers
#include <Arduino.h>
#include <Queue.h>

// Create the queue before creating tasks
QueueHandle_t myQueue;

void setup() {
    // Initialize the queue
    myQueue = xQueueCreate(10, sizeof(int));
    if (myQueue == NULL) {
        // Handle error: Queue could not be created
        Serial.println("Queue creation failed!");
        return;
    }
    
    // Create tasks
    xTaskCreate(Task1, "Task1", 1000, NULL, 1, NULL);
    xTaskCreate(Task2, "Task2", 1000, NULL, 1, NULL);
}

void Task1(void *pvParameters) {
    // Task implementation
}

void Task2(void *pvParameters) {
    // Task implementation
}