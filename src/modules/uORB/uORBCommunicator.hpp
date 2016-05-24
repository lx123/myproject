
#ifndef _uORBCommunicator_hpp_
#define _uORBCommunicator_hpp_

#include <stdint.h>


namespace uORBCommunicator  //名词空间，会话
{
class IChannel;
class IChannelRxHandler;  //接收处理
}

/**
 * Interface to enable remote subscriptions.  The implementor of this interface
 * shall manage the communication channel. It can be fastRPC or tcp or ip.  远程订阅接口，这个接口管理会话通道
 */

class uORBCommunicator::IChannel
{
public:

	//=========================================================================
	//     INTERFACES FOR Control messages over a channel.
	//=========================================================================

	/**
	 * @brief Interface to notify the remote entity of interest of a
	 * subscription for a message.
	 *
	 * @param messageName
	 * 	This represents the uORB message name; This message name should be
	 * 	globally unique.
	 * @param msgRate
	 * 	The max rate at which the subscriber can accept the messages.
	 * @return
	 * 	0 = success; This means the messages is successfully sent to the receiver
	 * 		Note: This does not mean that the receiver as received it.
	 *  otherwise = failure.
	 */

	virtual int16_t add_subscription(const char *messageName, int32_t msgRateInHz) = 0;



	/**
	 * @brief Interface to notify the remote entity of removal of a subscription
	 *
	 * @param messageName
	 * 	This represents the uORB message name; This message name should be
	 * 	globally unique.
	 * @return
	 * 	0 = success; This means the messages is successfully sent to the receiver
	 * 		Note: This does not necessarily mean that the receiver as received it.
	 *  otherwise = failure.
	 */

	virtual int16_t remove_subscription(const char *messageName) = 0;


	/**
	 * Register Message Handler.  This is internal for the IChannel implementer*注册消息句柄
	 */
	virtual int16_t register_handler(uORBCommunicator::IChannelRxHandler *handler) = 0;


	//=========================================================================
	//     INTERFACES FOR Data messages  数据管理接口
	//=========================================================================

	/**
	 * @brief Sends the data message over the communication link.
	 * @param messageName
	 * 	This represents the uORB message name; This message name should be
	 * 	globally unique.
	 * @param length
	 * 	The length of the data buffer to be sent.
	 * @param data
	 * 	The actual data to be sent.
	 * @return
	 *  0 = success; This means the messages is successfully sent to the receiver
	 * 		Note: This does not mean that the receiver as received it.
	 *  otherwise = failure.
	 */

	virtual int16_t send_message(const char *messageName, int32_t length, uint8_t *data) = 0;

};

/**
 * Class passed to the communication link implement to provide callback for received
 * messages over a channel.
 */
class uORBCommunicator::IChannelRxHandler
{
public:

	/**
	 * Interface to process a received AddSubscription from remote.
	 * @param messageName
	 * 	This represents the uORB message Name; This message Name should be
	 * 	globally unique.
	 * @param msgRate
	 * 	The max rate at which the subscriber can accept the messages.
	 * @return
	 *  0 = success; This means the messages is successfully handled in the
	 *  	handler.
	 *  otherwise = failure.
	 */

	virtual int16_t process_add_subscription(const char *messageName, int32_t msgRateInHz) = 0;


	/**
	 * Interface to process a received control msg to remove subscription
	 * @param messageName
	 * 	This represents the uORB message Name; This message Name should be
	 * 	globally unique.
	 * @return
	 *  0 = success; This means the messages is successfully handled in the
	 *  	handler.
	 *  otherwise = failure.
	 */

	virtual int16_t process_remove_subscription(const char *messageName) = 0;


	/**
	 * Interface to process the received data message.
	 * @param messageName
	 * 	This represents the uORB message Name; This message Name should be
	 * 	globally unique.
	 * @param length
	 * 	The length of the data buffer to be sent.
	 * @param data
	 * 	The actual data to be sent.
	 * @return
	 *  0 = success; This means the messages is successfully handled in the
	 *  	handler.
	 *  otherwise = failure.
	 */

	virtual int16_t process_received_message(const char *messageName, int32_t length, uint8_t *data) = 0;

};

#endif /* _uORBCommunicator_hpp_ */

