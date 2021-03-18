#include "MotionAdaptationToolV2HandlerFactory.h"
#include "MotionAdaptationToolV2Handler.h"

#include <boost/extension/extension.hpp>

using namespace MMM;

// register this factory
MotionHandlerFactory::SubClassRegistry MotionAdaptationToolV2HandlerFactory::registry(MotionAdaptationToolV2Handler::NAME, &MotionAdaptationToolV2HandlerFactory::createInstance);

MotionAdaptationToolV2HandlerFactory::MotionAdaptationToolV2HandlerFactory() : MotionHandlerFactory() {}

MotionAdaptationToolV2HandlerFactory::~MotionAdaptationToolV2HandlerFactory() = default;

std::string MotionAdaptationToolV2HandlerFactory::getName() {
    return MotionAdaptationToolV2Handler::NAME;
}

MotionHandlerPtr MotionAdaptationToolV2HandlerFactory::createMotionHandler(QWidget* widget) {
    return MotionHandlerPtr(new MotionAdaptationToolV2Handler(widget));
}

MotionHandlerFactoryPtr MotionAdaptationToolV2HandlerFactory::createInstance(void *) {
    return MotionHandlerFactoryPtr(new MotionAdaptationToolV2HandlerFactory());
}

extern "C"
BOOST_EXTENSION_EXPORT_DECL MotionHandlerFactoryPtr getFactory() {
    return MotionHandlerFactoryPtr(new MotionAdaptationToolV2HandlerFactory());
}

extern "C"
BOOST_EXTENSION_EXPORT_DECL std::string getVersion() {
    return MotionHandlerFactory::VERSION;
}
