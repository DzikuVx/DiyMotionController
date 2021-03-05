#ifndef MATH_H
#define MATH_H

typedef struct stdev_s
{
    float m_oldM, m_newM, m_oldS, m_newS;
    int m_n;
} stdev_t;

void devClear(stdev_t *dev);
void devPush(stdev_t *dev, float x);
float devVariance(stdev_t *dev);
float devStandardDeviation(stdev_t *dev);
float applyDeadband(float value, float deadband);
float fconstrainf(float amt, float low, float high);
float fscalef(float x, float srcMin, float srcMax, float destMin, float destMax);

#endif