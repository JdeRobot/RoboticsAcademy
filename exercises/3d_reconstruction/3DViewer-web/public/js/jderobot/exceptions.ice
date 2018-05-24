#ifndef EXCEPTIONS_ICE
#define EXCEPTIONS_ICE

module jderobot{

  exception JderobotException
  {
    //! Error description.
    string what;
  };
  
  //! Server failed to configure itself as requrested by client.
  exception ConfigurationNotExistException extends JderobotException {};

  /*!
    Raised when the server does not have the requested data.
    
    Typically, this is because the server has not fully initialized yet.
  */
  exception DataNotExistException extends JderobotException {};
  
  //! Indicates a problem with robot hardware, e.g. sensors and actuators.
  exception HardwareFailedException extends JderobotException {};

  //! Raised when the server is unable to return a topic for subscription.
  exception NoTopicException extends JderobotException {};

  //! Raised when the server fails to subscribe client for periodic updates.
  exception SubscriptionFailedException extends JderobotException {};

  //! Raised when the server fails to push initial data to a new subscriber.
  exception SubscriptionPushFailedException extends JderobotException {};

}; /*module*/

#endif /*EXCEPTIONS_ICE*/
