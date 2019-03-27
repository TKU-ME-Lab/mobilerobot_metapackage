如果載入STL檔時跳出以下訊息:
  It starts with the word 'solid', indicating that it's an ASCII STL file, 
  but it does not contain the word 'endsolid' soit is either a malformed ASCII STL file or it is actually a binary STL file. 
  Trying to interpret it as a binary STL file instead.

這個問題為STL裡的標題裡有solid字串，請用以下指令將solid字串替換成robot:
  
  sed -i 's/^solid/robot/' *

*號代表對所有檔案