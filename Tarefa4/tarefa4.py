"""
Autor: Lucas Cordeiro Butzke 

Codigo para simulacao de tarefas aperiodicas, 
verificar cumprimento de deadlines e projeto 
de escalonadores
"""
import sys
from simso.core import Model, Timer
from simso.configuration import Configuration


def mainEDF1():
    # Passar as tarefas atraves de um arquivo ou argumento adional
    configuration = Configuration()

    # Verificar tempo necessario de simulacao e quantas simulacoes realizar
    configuration.duration = 40 * configuration.cycles_per_ms

    # Adquirir tarefas do arquivo ou caso nao tiver por um default
    configuration.add_task(name="T1", identifier=1, period=3,
        activation_date=0, wcet=0.5, deadline=3)
    configuration.add_task(name="T2", identifier=2, period=4,
        activation_date=0, wcet=1.5, deadline=3)
    configuration.add_task(name="T3", identifier=3, period=7,
        activation_date=0, wcet=1, deadline=5)

    # Processadores do hardware de teste
    configuration.add_processor(name="CPU 1", identifier=1)

    # Realizar testes em RM e EDF, gerar um relatorio final das simulacoes
    #configuration.scheduler_info.filename = "examples/RM.py"
    configuration.scheduler_info.clas = "simso.schedulers.RM"

    # Check the config before trying to run it.
    configuration.check_all()

    # Init a model from the configuration.
    model = Model(configuration)

    # configuracao timer de uma funcao 
    funcTimer = Timer(model, None, 
                    (name="T4", identifier=4, period=30, activation_date=0, wcet=10, deadline=30),  # argumentos funcao timer
                    50, one_shot=False, prior=False, cpu=None, in_ms=False, overhead=0)

    funcTimer.Start()
    
    # Execute the simulation.
    model.run_model()

    # Print logs.
    for log in model.logs:
        print(log)


# simulacao 1
mainEDF1()
