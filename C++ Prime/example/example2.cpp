#include <iostream>
int main()
{
    int sum=0,value=0;
    while (std::cin >> value){
        sum+=value; //等价于sum=sum+value
        std::cout << "now Sum is" << sum << std::endl;

    }

    std::cout << "finally Sum is: " << sum << std::endl;
}