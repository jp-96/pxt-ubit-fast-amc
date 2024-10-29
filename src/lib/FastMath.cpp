#include <cstdint>

class FastMath
{
public:
    static float fastInverseSqrt(float number)
    {
        // float Q_rsqrt( float number )
        // https://github.com/id-Software/Quake-III-Arena/blob/master/code/game/q_math.c#L552
        // 
        // float invSqrt( float number )
        // https://stackoverflow.com/a/53893163
        // https://stackoverflow.com/questions/12923657/is-there-a-fast-c-or-c-standard-library-function-for-double-precision-inverse
        
        union
        {
            float f;
            uint32_t i;
        } conv;

        conv.f = number;
        conv.i = 0x5f3759df - (conv.i >> 1);
        return conv.f * (1.5F - (number * 0.5F * conv.f * conv.f));
    }
};
