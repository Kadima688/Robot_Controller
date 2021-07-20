#ifndef MhDH_H
#define MhDH_H
#include<iostream>
#include<vector>
namespace Mh{
    class MhDH{
        public:
            void set_link_number(int n);
            int get_link_number() const {return link_number;}
            void set_alapha(const std::vector<double>& value);
            void set_a(const std::vector<double>& value);
            void set_theta(const std::vector<double>& value);
            void set_d(const std::vector<double>& value);
            
        private:
            std::vector<double> alapha;
            std::vector<double> a;
            std::vector<double> theta;
            std::vector<double> d;
            int link_number;
    };
}
#endif