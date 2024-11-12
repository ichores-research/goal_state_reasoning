import os
from dotenv import load_dotenv

from langchain_core.prompts import PromptTemplate
from langchain_openai import ChatOpenAI


def main():
    load_dotenv() #load env vars from .env file
    
    post_template ="""
    Given this information {topic} write a short summary. What is the modt important note to remember? 
    """
    post_propmt_template = PromptTemplate(input_variables=["topic"], template=post_template)
    llm = ChatOpenAI(temperature=0, model_name="gpt-3.5-turbo")

    #Lang CHAIN
    chain = post_propmt_template | llm  #pipe operator comes from langchain 
    topic ="""
Jean-Henri d'Anglebert (baptized 1 April 1629 – 23 April 1691) was a French composer, harpsichordist and organist. He was one of the foremost keyboard composers of his day.

Life
D'Anglebert's father Claude Henry known as Anglebert[1] was an affluent shoemaker in Bar-le-Duc. Nothing is known about the composer's early years and musical education. Since he at one time composed a tombeau for Jacques Champion de Chambonnières, it is possible that Chambonnières was his teacher—or at any rate a friend for whom D'Anglebert had much respect. The earliest surviving manuscript with D'Anglebert's music dates from 1650–1659. It also contains music by Louis Couperin and Chambonnières, and possibly originated in their immediate circle; thus already by the mid-1650s D'Anglebert must have been closely associated with the most prominent French harpsichordists of the time. The earliest reference to D'Anglebert survives in his marriage contract from 11 October 1659. D'Anglebert married Magdelaine Champagne, sister-in-law of the organist François Roberday. In the contract, he is described as bourgeois de Paris, suggesting that by 1659 he was already well established in Paris. How he left Bar-le-Duc and settled in Paris remains unknown.
"""
    res = chain.invoke(input={"topic": topic})

    print(f'{res.content}')

if __name__ == "__main__":
    main()