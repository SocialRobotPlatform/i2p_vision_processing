/**
 * Autogenerated by Thrift Compiler (0.9.0-dev)
 *
 * DO NOT EDIT UNLESS YOU ARE SURE THAT YOU KNOW WHAT YOU ARE DOING
 *  @generated
 */
#ifndef SoundService_H
#define SoundService_H

#include <thrift/TDispatchProcessor.h>
#include "Inputs_types.h"

namespace imi {

class SoundServiceIf {
 public:
  virtual ~SoundServiceIf() {}
  virtual void sound(const std::string& sensorID, const  ::imi::Microseconds timestamp, const AudioLocalization& source) = 0;
};

class SoundServiceIfFactory {
 public:
  typedef SoundServiceIf Handler;

  virtual ~SoundServiceIfFactory() {}

  virtual SoundServiceIf* getHandler(const ::apache::thrift::TConnectionInfo& connInfo) = 0;
  virtual void releaseHandler(SoundServiceIf* /* handler */) = 0;
};

class SoundServiceIfSingletonFactory : virtual public SoundServiceIfFactory {
 public:
  SoundServiceIfSingletonFactory(const boost::shared_ptr<SoundServiceIf>& iface) : iface_(iface) {}
  virtual ~SoundServiceIfSingletonFactory() {}

  virtual SoundServiceIf* getHandler(const ::apache::thrift::TConnectionInfo&) {
    return iface_.get();
  }
  virtual void releaseHandler(SoundServiceIf* /* handler */) {}

 protected:
  boost::shared_ptr<SoundServiceIf> iface_;
};

class SoundServiceNull : virtual public SoundServiceIf {
 public:
  virtual ~SoundServiceNull() {}
  void sound(const std::string& /* sensorID */, const  ::imi::Microseconds /* timestamp */, const AudioLocalization& /* source */) {
    return;
  }
};

typedef struct _SoundService_sound_args__isset {
  _SoundService_sound_args__isset() : sensorID(false), timestamp(false), source(false) {}
  bool sensorID;
  bool timestamp;
  bool source;
} _SoundService_sound_args__isset;

class SoundService_sound_args {
 public:

  SoundService_sound_args() : sensorID(), timestamp(0) {
  }

  virtual ~SoundService_sound_args() throw() {}

  std::string sensorID;
   ::imi::Microseconds timestamp;
  AudioLocalization source;

  _SoundService_sound_args__isset __isset;

  void __set_sensorID(const std::string& val) {
    sensorID = val;
  }

  void __set_timestamp(const  ::imi::Microseconds val) {
    timestamp = val;
  }

  void __set_source(const AudioLocalization& val) {
    source = val;
  }

  bool operator == (const SoundService_sound_args & rhs) const
  {
    if (!(sensorID == rhs.sensorID))
      return false;
    if (!(timestamp == rhs.timestamp))
      return false;
    if (!(source == rhs.source))
      return false;
    return true;
  }
  bool operator != (const SoundService_sound_args &rhs) const {
    return !(*this == rhs);
  }

  bool operator < (const SoundService_sound_args & ) const;

  uint32_t read(::apache::thrift::protocol::TProtocol* iprot);
  uint32_t write(::apache::thrift::protocol::TProtocol* oprot) const;

};


class SoundService_sound_pargs {
 public:


  virtual ~SoundService_sound_pargs() throw() {}

  const std::string* sensorID;
  const  ::imi::Microseconds* timestamp;
  const AudioLocalization* source;

  uint32_t write(::apache::thrift::protocol::TProtocol* oprot) const;

};

class SoundServiceClient : virtual public SoundServiceIf {
 public:
  SoundServiceClient(boost::shared_ptr< ::apache::thrift::protocol::TProtocol> prot) :
    piprot_(prot),
    poprot_(prot) {
    iprot_ = prot.get();
    oprot_ = prot.get();
  }
  SoundServiceClient(boost::shared_ptr< ::apache::thrift::protocol::TProtocol> iprot, boost::shared_ptr< ::apache::thrift::protocol::TProtocol> oprot) :
    piprot_(iprot),
    poprot_(oprot) {
    iprot_ = iprot.get();
    oprot_ = oprot.get();
  }
  boost::shared_ptr< ::apache::thrift::protocol::TProtocol> getInputProtocol() {
    return piprot_;
  }
  boost::shared_ptr< ::apache::thrift::protocol::TProtocol> getOutputProtocol() {
    return poprot_;
  }
  void sound(const std::string& sensorID, const  ::imi::Microseconds timestamp, const AudioLocalization& source);
  void send_sound(const std::string& sensorID, const  ::imi::Microseconds timestamp, const AudioLocalization& source);
 protected:
  boost::shared_ptr< ::apache::thrift::protocol::TProtocol> piprot_;
  boost::shared_ptr< ::apache::thrift::protocol::TProtocol> poprot_;
  ::apache::thrift::protocol::TProtocol* iprot_;
  ::apache::thrift::protocol::TProtocol* oprot_;
};

class SoundServiceProcessor : public ::apache::thrift::TDispatchProcessor {
 protected:
  boost::shared_ptr<SoundServiceIf> iface_;
  virtual bool dispatchCall(apache::thrift::protocol::TProtocol* iprot, apache::thrift::protocol::TProtocol* oprot, const std::string& fname, int32_t seqid, void* callContext);
 private:
  typedef  void (SoundServiceProcessor::*ProcessFunction)(int32_t, apache::thrift::protocol::TProtocol*, apache::thrift::protocol::TProtocol*, void*);
  typedef std::map<std::string, ProcessFunction> ProcessMap;
  ProcessMap processMap_;
  void process_sound(int32_t seqid, apache::thrift::protocol::TProtocol* iprot, apache::thrift::protocol::TProtocol* oprot, void* callContext);
 public:
  SoundServiceProcessor(boost::shared_ptr<SoundServiceIf> iface) :
    iface_(iface) {
    processMap_["sound"] = &SoundServiceProcessor::process_sound;
  }

  virtual ~SoundServiceProcessor() {}
};

class SoundServiceProcessorFactory : public ::apache::thrift::TProcessorFactory {
 public:
  SoundServiceProcessorFactory(const ::boost::shared_ptr< SoundServiceIfFactory >& handlerFactory) :
      handlerFactory_(handlerFactory) {}

  ::boost::shared_ptr< ::apache::thrift::TProcessor > getProcessor(const ::apache::thrift::TConnectionInfo& connInfo);

 protected:
  ::boost::shared_ptr< SoundServiceIfFactory > handlerFactory_;
};

class SoundServiceMultiface : virtual public SoundServiceIf {
 public:
  SoundServiceMultiface(std::vector<boost::shared_ptr<SoundServiceIf> >& ifaces) : ifaces_(ifaces) {
  }
  virtual ~SoundServiceMultiface() {}
 protected:
  std::vector<boost::shared_ptr<SoundServiceIf> > ifaces_;
  SoundServiceMultiface() {}
  void add(boost::shared_ptr<SoundServiceIf> iface) {
    ifaces_.push_back(iface);
  }
 public:
  void sound(const std::string& sensorID, const  ::imi::Microseconds timestamp, const AudioLocalization& source) {
    size_t sz = ifaces_.size();
    size_t i = 0;
    for (; i < (sz - 1); ++i) {
      ifaces_[i]->sound(sensorID, timestamp, source);
    }
    ifaces_[i]->sound(sensorID, timestamp, source);
  }

};

} // namespace

#endif
