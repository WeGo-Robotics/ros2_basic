#include <pluginlib/class_loader.hpp>
#include <calculator/calculator.hpp>
#include <iostream>

using namespace std;

int main(int argc, char ** argv)
{
    (void) argc;
    (void) argv;

    pluginlib::ClassLoader<calculator_base::RegularCalculator> cal_loader("calculator", "calculator_base::RegularCalculator");

    std::shared_ptr<calculator_base::RegularCalculator> add = cal_loader.createSharedInstance("calculator_plugins::Add");
    add->write_number(2.0, 2.0);

    std::shared_ptr<calculator_base::RegularCalculator> sub = cal_loader.createSharedInstance("calculator_plugins::Sub");
    sub->write_number(2.0, 2.0);

    std::shared_ptr<calculator_base::RegularCalculator> mul = cal_loader.createSharedInstance("calculator_plugins::Mul");
    mul->write_number(2.0, 2.0);

    std::shared_ptr<calculator_base::RegularCalculator> div = cal_loader.createSharedInstance("calculator_plugins::Div");
    div->write_number(2.0, 2.0);

    cout << "Add: " << add->calculate() << endl
         << "Sub: " << sub->calculate() << endl
         << "Mul: " << mul->calculate() << endl
         << "Div: " << div->calculate() << endl;

    return 0;
}