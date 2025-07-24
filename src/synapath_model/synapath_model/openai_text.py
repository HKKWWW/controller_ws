# 标准库
import sys
import os

# 自定义库
from synapath.models.largeModel.vlms.openaiAgent import openaiAgent
from synapath.models.largeModel.utils_LLM import print_dialog

if __name__ == "__main__":

    # 获取当前工作目录 C:\work\Synapath_Python_Platform\models\largeModel\examples\qwen.yaml
    current_working_directory = os.getcwd()
    print(f"当前工作目录: {current_working_directory}")

    # 获取配置文件路径
    cfg_path = os.path.join(current_working_directory, 
                            "src",
                            "synapath_model",
                            "synapath_model",
                            "qwen.yaml")
    print("------------------------\n", cfg_path)
   
    # 创建一个OpenAI代理
    agent = openaiAgent(cfg_path)

    question = "你现在扮演奶龙"
    answer = agent(question, use_history=True)

    print(f'AI 回答: {answer}')

    agent._append_history(question, answer)  # 添加历史
    # print(agent.history)

    question = "你在扮演谁"
    answer = agent(question, use_history=True)
    print(f'AI 回答: {answer}')

    agent._append_history(question, answer)  # 添加历史

    # 检查历史记录
    from rich.console import Console  # 输出美化
    console = Console()
    console.print("所有的历史记录: \n")
    console.print(agent.history)
    