#include <calculator/calculator.hpp>

namespace calculator_plugins
{
    class Add : public calculator_base::RegularCalculator
    {
        public:
            void write_number(double num1, double num2) override
            {
                num_a = num1;
                num_b = num2;
            }

            double calculate() override
            {
                return num_a + num_b;
            }

        protected:
            double num_a, num_b;
    };

    class Sub : public calculator_base::RegularCalculator
    {
        public:
            void write_number(double num1, double num2) override
            {
                num_a = num1;
                num_b = num2;
            }

            double calculate() override
            {
                return num_a - num_b;
            }

        protected:
            double num_a, num_b;
    };

    class Mul : public calculator_base::RegularCalculator
    {
        public:
            void write_number(double num1, double num2) override
            {
                num_a = num1;
                num_b = num2;
            }

            double calculate() override
            {
                return num_a * num_b;
            }

        protected:
            double num_a, num_b;
    };

    class Div : public calculator_base::RegularCalculator
    {
        public:
            void write_number(double num1, double num2) override
            {
                num_a = num1;
                num_b = num2;
            }

            double calculate() override
            {
                if(num_b == 0)
                    return 0;
                else
                    return num_a / num_b;
            }

        protected:
            double num_a, num_b;
    };
}

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(calculator_plugins::Add, calculator_base::RegularCalculator)
PLUGINLIB_EXPORT_CLASS(calculator_plugins::Sub, calculator_base::RegularCalculator)
PLUGINLIB_EXPORT_CLASS(calculator_plugins::Mul, calculator_base::RegularCalculator)
PLUGINLIB_EXPORT_CLASS(calculator_plugins::Div, calculator_base::RegularCalculator)