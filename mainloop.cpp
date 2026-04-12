int main() {
    // Standard Init and TinyUSB setup...
    setup_hardware_safety(pwm_slice, 10000); // 1-second fuse at 10kHz

    while (true) {
        tud_task(); // Your USB activity

        // Poll the hardware fuse status
        if (!dma_channel_is_busy(count_chan) && !dma_channel_is_busy(kill_chan)) {
            // THE FUSE HAS TRIPPED
            // Handle the runaway event (log to USB, alert user)
            uint32_t health_check = dma_sniffer_get_data();
        }

        // Reset the counter if activity is detected
        if (usb_activity_detected()) {
             rearm_safety(count_chan, kill_chan, pwm_slice, 10000);
        }
    }
}

