package main

import (
	"flag"
	"io"
	"log"
	"net"
	"os"
	"os/signal"
	"syscall"
)

func main() {
	// Ability to set address and port from command args (go main.go --addr=<address>)
	addr := flag.String("addr", ":9000", "listen address (host:port)")
	flag.Parse()
	// Add better log prefix
	log.SetFlags(log.LstdFlags | log.Lmicroseconds)

	// Start a TCP socket listenning on defined address
	ln, err := net.Listen("tcp", *addr)
	if err != nil {
		log.Fatalf("listen error: %v", err)
	}
	defer ln.Close()
	log.Printf("listening on %s", *addr)

	// Graceful shutdown when force closing the program
	sig := make(chan os.Signal, 1)
	signal.Notify(sig, syscall.SIGINT, syscall.SIGTERM)
	go func() {
		<-sig
		log.Println("shutting down listener")
		_ = ln.Close()
	}()

	// Main loop that waits for a client to connect
	// to the socket and initiate the connection
	for {
		conn, err := ln.Accept()
		if err != nil {
			log.Printf("accept: %v", err)
			return
		}
		log.Printf("client connected from %s", conn.RemoteAddr())
		go handleConn(conn)
	}
}

func handleConn(conn net.Conn) {
	defer func() {
		log.Printf("client disconnected %s", conn.RemoteAddr())
		_ = conn.Close()
	}()

	// Pipe: socket -> stdout
	go func() {
		_, _ = io.Copy(os.Stdout, conn)
	}()

	// Pipe: stdin -> socket
	_, _ = io.Copy(conn, os.Stdin)

	// If stdin hits EOF, close write side so the peer sees it.
	type closeWriter interface{ CloseWrite() error }
	if cw, ok := conn.(closeWriter); ok {
		_ = cw.CloseWrite()
	}
}
