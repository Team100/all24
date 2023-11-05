package org.team100.lib.random;

import java.util.Random;

public class MersenneTwister extends Random {
    private static final int N = 624;
    private static final int M = 397;
    private static final int UPPER_MASK = 0x80000000;
    private static final int LOWER_MASK = 0x7fffffff;
    private static final int[] MAG01 = { 0, 0x9908b0df };
    private int[] _state;
    private int _index;

    public MersenneTwister(int seed) {
        setSeed(seed);
    }

    @Override
    public void setSeed(long seed) {
        _state = new int[N];
        _state[0] = (int) seed;
        for (_index = 1; _index < N; _index++) {
            _state[_index] = 1812433253
                    * (_state[_index - 1] ^ (_state[_index - 1] >>> 30))
                    + _index;
        }
    }

    @Override
    protected int next(int bits) {
        if (_index >= N) {
            int kk;
            for (kk = 0; kk < N - M; ++kk) {
                int y = (_state[kk] & UPPER_MASK) | (_state[kk + 1] & LOWER_MASK);
                _state[kk] = _state[kk + M] ^ (y >>> 1) ^ MAG01[y & 1];
            }
            for (; kk < N - 1; ++kk) {
                int y = (_state[kk] & UPPER_MASK) | (_state[kk + 1] & LOWER_MASK);
                _state[kk] = _state[kk + (M - N)] ^ (y >>> 1) ^ MAG01[y & 1];
            }
            int y = (_state[N - 1] & UPPER_MASK) | (_state[0] & LOWER_MASK);
            _state[N - 1] = _state[M - 1] ^ (y >>> 1) ^ MAG01[y & 1];
            _index = 0;
        }
        int y = _state[_index++];
        y ^= (y >>> 11);
        y ^= (y << 7) & 0x9d2c5680;
        y ^= (y << 15) & 0xefc60000;
        y ^= (y >>> 18);
        return y >>> (32 - bits);
    }
}
