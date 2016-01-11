import pygame
from pygame.locals import *
pygame.init()
continuer = 1
while continuer:
	for event in pygame.event.get():	#Atten
		if event.type == KEYDOWN:
    			 if event.key == K_SPACE:
          		 	print("Espace")
     			 if event.key == K_RETURN:
           		 	print("Entree")
