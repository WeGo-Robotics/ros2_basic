#pragma once

namespace calculator_base
{
    class RegularCalculator
    {
        public:
            virtual void write_number(double num1, double num2) = 0;
            virtual double calculate() = 0;
            virtual ~RegularCalculator() = default;
        protected:
            RegularCalculator()=default;
    };
}