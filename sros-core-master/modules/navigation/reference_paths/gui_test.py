from tkinter import *

def callback():
    print('**点击按钮**')

root=Tk()
root.geometry("400x400+200+200")    #对应的格式为宽乘以高加上水平偏移量加上垂直偏移量

#定义lable对象用Lable方法，顺序分别为窗口对象，显示文本python程序设计,字体内型为华文行楷，大小为20
#字体颜色为绿色，背景颜色为粉色
# label=Label(root, text="Python程序设计", font=("华文行楷", 20),fg="green", bg="pink")
# label.pack()    #调用pack方法将label标签显示在主界面，后面也会用到就不一一解释了

button=Button(root, text='**点击按钮**', command=callback)
button.pack()

data=StringVar(value='default')    #创建可编数据data
label=Label(root, textvariable=data)    #创建label组件并将其与data关联
label.pack()
entry =Entry(root, textvariable=data)    #创建labal组件并将其与data关联
entry.pack()

#root.mainloop()

if __name__ == "__main__":
    print ('This is main of module "hello.py"')
    root.mainloop()