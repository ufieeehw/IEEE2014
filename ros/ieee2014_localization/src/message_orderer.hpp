#ifndef GUARD_UPOZXSDEHOQEZPUK
#define GUARD_UPOZXSDEHOQEZPUK


class MessageOrderer {
private:
  class ITopic {
  public:
    virtual ~ITopic() {
    }
    virtual boost::optional<ros::Time> getStamp() const = 0;
    virtual void popMessage() = 0;
  };
  template<typename MessageType>
  class Topic : public ITopic {
    boost::function<void(MessageType const &)> callback;
    ros::Subscriber sub;
    std::list<MessageType> queue;
  public:
    Topic(ros::NodeHandle &nh, MessageOrderer &mo, std::string const &topic, boost::function<void(MessageType const &)> callback) :
      callback(callback) {
      sub = nh.subscribe(topic, 10, boost::function<void(boost::shared_ptr<MessageType const> const &)>([this, &mo](boost::shared_ptr<MessageType const> const &msgp) {
        if(!queue.empty()) {
          ROS_ASSERT(msgp->header.stamp >= queue.back().header.stamp);
        }
        queue.push_back(*msgp);
        mo.process();
      }));
    }
    boost::optional<ros::Time> getStamp() const {
      if(queue.empty()) {
        return boost::none;
      } else {
        return queue.front().header.stamp;
      }
    }
    void popMessage() {
      callback(queue.front());
      queue.pop_front();
    }
  };

public:
  class ITopicDescriptor {
  public:
    virtual ~ITopicDescriptor() {
    }
    virtual std::unique_ptr<ITopic> makeITopic(ros::NodeHandle &nh, MessageOrderer &mo) const = 0;
  };
  template<typename MessageType>
  class TopicDescriptor : public ITopicDescriptor {
    std::string topic;
    boost::function<void(MessageType const &)> callback;
  public:
    TopicDescriptor(std::string const &topic, boost::function<void(MessageType const &)> callback) :
      topic(topic), callback(callback) {
    }
    std::unique_ptr<ITopic> makeITopic(ros::NodeHandle &nh, MessageOrderer &mo) const {
      return std::unique_ptr<ITopic>(new Topic<MessageType>(nh, mo, topic, callback));
    }
  };

private:
  std::vector<std::unique_ptr<ITopic> > topics;

public:
  MessageOrderer(ros::NodeHandle &nh, std::vector<std::unique_ptr<ITopicDescriptor> > const &topicdescriptors) {
    BOOST_FOREACH(std::unique_ptr<ITopicDescriptor> const &tdp, topicdescriptors) {
      topics.push_back(std::move(tdp->makeITopic(nh, *this)));
    }
  }
  
  void process() {
    while(true) {
      ITopic *best_topic = nullptr;
      BOOST_FOREACH(std::unique_ptr<ITopic> const &tp, topics) {
        if(!tp->getStamp()) {
          return;
        }
        if(!best_topic || *tp->getStamp() < *best_topic->getStamp()) {
          best_topic = tp.get();
        }
      }
      best_topic->popMessage();
    }
  }
};


#endif
