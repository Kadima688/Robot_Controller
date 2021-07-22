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
            inline std::vector<double> get_alapha(){return alapha;};
            inline std::vector<double> get_a(){return a;};
            inline std::vector<double> get_theta(){return theta;};
            inline std::vector<double> get_d(){return d;};
            void show_dh_table();
            
        protected:
            std::vector<double> alapha;
            std::vector<double> a;
            std::vector<double> theta;
            std::vector<double> d;
            int link_number;
    };
}
#endif