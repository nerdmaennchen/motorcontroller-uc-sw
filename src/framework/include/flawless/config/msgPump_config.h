#pragma once


/**
 * the size of the msg queue
 * this reflects the amount of messages that can be posted at the same time
 */
#define MSG_PUMP_MSG_QUEUE_MAX_SIZE 128


/*
 * some hex number should be mostly unique and must fit into an integer
 */
#define MSG_PUMP_MAGIC 0xDEADBEEF

/*
 * use magic number in header or not
 * (if defined the header magic will be used)
 */
#define MSG_PUMP_USE_HEADER_MAGIC
