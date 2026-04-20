def parse_data(self, data):
    self.recv_buffer.extend(data)
    if len(self.recv_buffer) > 1024:
        self.recv_buffer.clear()
        return
    while len(self.recv_buffer) >= 5:
        if self.recv_buffer[0] != 0xBB:
            self.recv_buffer.pop(0)
            continue
        func = self.recv_buffer[1]
        d_len = self.recv_buffer[2]
        if d_len > 255 or d_len < 0:
            self.recv_buffer.pop(0)
            continue
        f_len = 3 + d_len + 1
        if len(self.recv_buffer) < f_len:
            break
        frame = self.recv_buffer[:f_len]
        self.recv_buffer = self.recv_buffer[f_len:]
        checksum_calc = sum(frame[:-1]) & 0xFF
        checksum_recv = frame[-1]
        if func == 0x02:
            payload = frame[3:-1]
            if len(payload) >= 2:
                esp_m, esp_s = payload[0], payload[1]
                needs_rebuild = False
                if esp_m != self.num_m:
                    self.num_m = esp_m
                    needs_rebuild = True
                if esp_s != self.num_s:
                    self.num_s = esp_s
                    needs_rebuild = True
                if needs_rebuild:
                    self.rebuild_cards()
                offset = 2
                new_motor = []
                new_states = []
                for j in range(esp_m):
                    if offset + 7 <= len(payload):
                        x, y, z = struct.unpack_from('>hhh', payload, offset)
                        offset += 6
                        state = payload[offset]
                        offset += 1
                        new_motor.append([x/100, y/100, z/100])
                        new_states.append(state)
                    else:
                        break
                self.update_filter_buffers_count()   # 确保缓冲区数量匹配
                filtered_motor = []
                for idx, (pos, vel, acc) in enumerate(new_motor):
                    fpos, fvel, facc = self.apply_filters_to_motor(idx, pos, vel, acc)
                    filtered_motor.append([fpos, fvel, facc])
                self.motor_data = filtered_motor
                self.motor_states = new_states

                new_sensor = []
                for j in range(esp_s):
                    if offset + 6 <= len(payload):
                        x, y, z = struct.unpack_from('>hhh', payload, offset)
                        offset += 6
                        new_sensor.append([x/100, y/100, z/100])
                    else:
                        break
                while len(new_sensor) < self.num_s:
                    new_sensor.append([0.0, 0.0, 0.0])

                filtered_sensor = []
                for idx, (pitch, roll, yaw) in enumerate(new_sensor):
                    fp, fr, fy = self.apply_filters_to_sensor(idx, pitch, roll, yaw)
                    filtered_sensor.append([fp, fr, fy])
                self.sensor_data = filtered_sensor


                if offset + 5 <= len(payload):
                    try:
                        scale1 = struct.unpack_from('>h', payload, offset)[0] / 100
                        offset += 2
                        scale2 = struct.unpack_from('>h', payload, offset)[0] / 100
                        offset += 2
                        sys_state = payload[offset]
                        self.scale_data = scale1
                        self.bend_angle = scale2
                    except Exception:
                        pass

                self.area_change = self.scale_data
                self.update_ui()