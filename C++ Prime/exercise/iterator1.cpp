#include <iostream>
#include <iostream>
#include <vector>



int main()
{
    std::vector<int> Zs;
    int z;
    while (std::cin >> z){
        Zs.push_back(z);
        // if (z==-1){
        //     std::cout << "输入元素操作结束 " << std::endl;
        //     break;
        // }
        
    }

    for (int i=0;i<Zs.size()-1;i++){
        std::cout << "相邻整数的和： " << Zs[i]+Zs[i+1] << std::endl;
    }

    for (int j=0;j<Zs.size()/2+1;j++){
        std::cout << "第" << j << "个元素和第" 
                << Zs.size()-j << "个元素之和是："
                << Zs[j]+Zs[Zs.size()-1-j] 
                << std::endl;
    }
}
