// Обёртка над SocketCAN: один RAW-сокет, фоновый поток приёма, потокобезопасная отправка.
#pragma once

#include <linux/can.h>

#include <atomic>
#include <functional>
#include <mutex>
#include <string>
#include <thread>

namespace vesc_nexus {

class CanInterface {
public:
  using FrameCallback = std::function<void(const can_frame&)>;

  explicit CanInterface(std::string interface_name);
  ~CanInterface();

  CanInterface(const CanInterface&) = delete;
  CanInterface& operator=(const CanInterface&) = delete;

  /// Открыть сокет и запустить поток приёма. Повторный вызов на открытом интерфейсе — no-op, true.
  bool open();

  /// Остановить поток приёма и закрыть сокет. Безопасно вызывать повторно.
  void close();

  /// Отправить фрейм. false — интерфейс закрыт или write() не удался (ошибка логируется с троттлингом).
  bool sendFrame(const can_frame& frame);

  /// Колбэк вызывается из потока приёма. Устанавливать до open().
  void setReceiveCallback(FrameCallback callback);

  bool isOpen() const { return running_; }
  const std::string& name() const { return interface_name_; }

private:
  void receiveLoop();

  std::string interface_name_;
  int socket_{-1};
  std::atomic<bool> running_{false};
  FrameCallback callback_;
  std::thread receive_thread_;
  std::mutex send_mutex_;
};

}  // namespace vesc_nexus
