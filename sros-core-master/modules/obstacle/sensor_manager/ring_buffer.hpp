//
// Created by lfc on 2020/9/2.
//

#ifndef SROS_RING_BUFFER_HPP
#define SROS_RING_BUFFER_HPP

namespace sensor {
class ring_buffer_t {
 public:

    char *buffer;
    int buffer_size;
    int first;
    int last;

    void ring_clear() {
        first = 0;
        last = 0;
    }

    void ring_initialize(char *buffer, const int shift_length) {
        buffer = buffer;
        buffer_size = 1 << shift_length;
        ring_clear();
    }

    int ring_size() { return (last >= first) ? last - first : buffer_size - (first - last); }

    int ring_capacity() { return buffer_size - 1; }

    static void byte_move(char *dest, const char *src, int n) {
        const char *last_p = dest + n;
        while (dest < last_p) {
            *dest++ = *src++;
        }
    }

    int ring_write(const char *data, int size) {
        int free_size = ring_capacity() - ring_size();
        int push_size = (size > free_size) ? free_size : size;

        // �f�[�^�z�u
        if (first <= last) {
            // last ���� buffer_size �I�[�܂łɔz�u
            int left_size = 0;
            int to_end = buffer_size - last;
            int move_size = (to_end > push_size) ? push_size : to_end;

            byte_move(&buffer[last], data, move_size);
            last += move_size;
            last &= (buffer_size - 1);

            left_size = push_size - move_size;
            if (left_size > 0) {
                // 0 ���� first �̑O�܂ł�z�u
                byte_move(buffer, &data[move_size], left_size);
                last = left_size;
            }
        } else {
            // last ���� first �̑O�܂Ŕz�u
            byte_move(&buffer[last], data, size);
            last += push_size;
        }
        return push_size;
    }

    int ring_read(char *buffer, int size) {
        // �f�[�^�擾
        int now_size = ring_size();
        int pop_size = (size > now_size) ? now_size : size;

        if (first <= last) {
            byte_move(buffer, &buffer[first], pop_size);
            first += pop_size;

        } else {
            // first ���� buffer_size �I�[�܂ł�z�u
            int left_size = 0;
            int to_end = buffer_size - first;
            int move_size = (to_end > pop_size) ? pop_size : to_end;
            byte_move(buffer, &buffer[first], move_size);

            first += move_size;
            first &= (buffer_size - 1);

            left_size = pop_size - move_size;
            if (left_size > 0) {
                // 0 ���� last �̑O�܂ł�z�u
                byte_move(&buffer[move_size], buffer, left_size);

                first = left_size;
            }
        }
        return pop_size;
    }
};
}  // namespace sensor

#endif  // SROS_RING_BUFFER_HPP
