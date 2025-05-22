"""
# PIP's
pip install -U discord.py
pip install python-dotenv
"""

import os
from dotenv import load_dotenv
import discord
from discord.ext import commands
load_dotenv()

token = os.getenv('DISCORD_TOKEN')

intents = discord.Intents.default()
intents.message_content = True  # Needed for reading messages

bot = commands.Bot(command_prefix='!', intents=intents)

@bot.event
async def on_ready():
    print(f'✅ Logged in as {bot.user} (ID: {bot.user.id})')

@bot.command()
async def hello(ctx):
    await ctx.send("Hello there!")

# Run the bot (replace with your token)
bot.run(token)
