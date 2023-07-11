import parameters as para

###############
locx=para.START_POS
print(locx[0])
##############

filename = "/home/timtu/桌面/summer program/avoid obstacle/Astar/parameters.py"  # 替換為你要修改的檔案路徑

# 讀取原始檔案
with open(filename, "r") as file:
    content = file.read()

# 修改數字部分
new_content = content.replace("(6,5)", "(10,10)")  # 將 (6,5) 替換為新的數值 (10,10)
new_content = new_content.replace("(19, 17)", "(20, 20)")  # 將 (19, 17) 替換為新的數值 (20, 20)
# 在這裡繼續進行其他需要的替換操作...

# 將修改後的內容寫入回原始檔案
with open(filename, "w") as file:
    file.write(new_content)

print("已成功修改檔案")


#################
locx=para.START_POS
print(locx[0])
#################
