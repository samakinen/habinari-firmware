#pragma once
#include <functional>
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "TPUart/Interface/Abstract.h"
#include "knx_tp_bit_bang.h"

namespace TPUart
{
    namespace Interface
    {
        class KnxTpBitBangInterface : public TPUart::Interface::Abstract
        {
          protected:
            bool _running = false;

          public:
            virtual void flush() override;
            virtual void begin(int baud) override;
            virtual void end() override;
            virtual bool available() override;
            virtual bool availableForWrite() override;
            virtual bool write(char value) override;
            virtual int read() override;
            virtual bool overflow() override;
            virtual bool hasCallback() override;
            virtual void registerCallback(std::function<bool()> callback) override;

            // Register task handle for direct ISR notifications (call before begin())
            void setTaskHandle(TaskHandle_t handle);
            TaskHandle_t getTaskHandle() const;
            bool hasMoreToProcess() const;
            knx_event_t getLastEvent();

        };
    } // namespace Interface
} // namespace TPUart