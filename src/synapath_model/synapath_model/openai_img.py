# 标准库
import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "../../../")))

# 第三方库
from PIL import Image

# 自定义库
from synapath.models.largeModel.vlms.openaiAgent import openaiAgent

if __name__ == "__main__":
    
    # 获取当前工作目录 
    current_working_directory = os.getcwd()
    print(f"当前工作目录: {current_working_directory}")

    # 获取配置文件路径
    cfg_path = os.path.join(current_working_directory, 
                            "src",
                            "synapath_model",
                            "synapath_model",
                            "qwen.yaml")    
    # 获取图片文件路径
    img_path = os.path.join(current_working_directory, 
                            "src",
                            "synapath_model",
                            "synapath_model",
                            "example.jpg")
    
    # 创建一个openaiAgent对象
    agent = openaiAgent(cfg_path)

    # 使用图片
    with Image.open(img_path) as pil_img:
    
        # 第1轮提问
        question = "画面里有什么"   
        answer = agent(question, pil_img, use_history=True)

        print(f'\n用户提问: {question}\n')
        print(f'\nAI 回答: {answer}\n')

        # 添加历史
        agent._append_history(question, answer)
        # print(agent.history)

        # 第2轮提问
        question = "说出画面里各物体的位置"
        answer = agent(question, use_history=True)

        print(f'\n用户提问: {question}\n')
        print(f'\nAI 回答: {answer}\n')
