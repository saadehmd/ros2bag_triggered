#ifndef MESSAGE_TYPE_REGISTRY_HPP
#define MESSAGE_TYPE_REGISTRY_HPP

#include <unordered_map>
#include <functional>
#include <memory>
#include <string>
#include <stdexcept>

class MessageTypeRegistry {
public:
    using CreateFunction = std::function<std::shared_ptr<void>()>;

    template <typename MsgType>
    void registerType(const std::string& type_name) {
        registry_[type_name] = []() -> std::shared_ptr<void> {
            return std::make_shared<MsgType>();
        };
    }

    std::shared_ptr<void> createMessage(const std::string& type_name) const {
        auto it = registry_.find(type_name);
        if (it != registry_.end()) {
            return it->second();
        }
        throw std::runtime_error("Message type not registered: " + type_name);
    }

private:
    std::unordered_map<std::string, CreateFunction> registry_;
};

#endif // MESSAGE_TYPE_REGISTRY_HPP