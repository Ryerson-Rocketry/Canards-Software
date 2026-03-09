#include "main.h"
#include "cmsis_os2.h"

osThreadId_t readSensorTaskHandle;
osThreadId_t altEstTaskHandle;
osThreadId_t launchDetTaskHandle;
osThreadId_t dataStoreTaskHandle;
osThreadId_t gpsRetrieveTaskHandle;
osThreadId_t heartbeatTaskHandle;
osThreadId_t controlTaskHandle;

const osThreadAttr_t readSensorTask_attributes = {
    .name = "readSensorTask", .stack_size = 1024 * 2, .priority = osPriorityAboveNormal6};
const osThreadAttr_t altEstTask_attributes = {
    .name = "altTask", .stack_size = 512 * 2, .priority = osPriorityAboveNormal4};
const osThreadAttr_t launchDetTask_attributes = {
    .name = "launchTask", .stack_size = 256 * 2, .priority = osPriorityAboveNormal2};
const osThreadAttr_t dataStoreTask_attributes = {
    .name = "storeTask", .stack_size = 1024 * 4, .priority = osPriorityNormal};
const osThreadAttr_t gpsRetrieveTask_attributes = {
    .name = "retrieveGpsCoords", .stack_size = 256 * 2, .priority = osPriorityAboveNormal1};
const osThreadAttr_t controlTask_attributes = {
    .name = "controlCanards", .stack_size = 512 * 2, .priority = osPriorityAboveNormal3};
const osThreadAttr_t heartbeat_attributes = {
    .name = "wdgTask", .stack_size = 256 * 2, .priority = osPriorityBelowNormal3};