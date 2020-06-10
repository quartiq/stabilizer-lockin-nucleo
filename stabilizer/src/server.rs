use heapless::{
    consts::*,
    String,
    Vec
};

use core::fmt::Write;


use serde::{
    Deserialize,
    Serialize
};

use serde_json_core::{
    de::from_slice,
    ser::to_string
};

use super::net;
use super::iir;

#[derive(Deserialize, Serialize, Debug)]
pub enum AccessRequest {
    Read,
    Write,
}

#[derive(Deserialize, Serialize, Debug)]
pub struct Request<'a> {
    pub req: AccessRequest,
    pub attribute: &'a str,
    pub value: String<U256>,
}

#[derive(Serialize, Deserialize)]
pub struct IirRequest {
    pub channel: u8,
    pub iir: iir::IIR,
}

#[derive(Serialize)]
pub struct Response {
    code: i32,
    attribute: String<U256>,
    value: String<U256>,
}

impl<'a> Request<'a> {
    pub fn restore_value(&mut self) {
        let mut new_value: String<U256> = String::new();
        for byte in self.value.as_str().chars() {
            if byte == '\'' {
                new_value.push('"').unwrap();
            } else {
                new_value.push(byte).unwrap();
            }
        }

        self.value = new_value;
    }
}

impl Response {

    fn sanitize_value(&mut self) {
        let mut new_value: String<U256> = String::new();
        for byte in self.value.as_str().chars() {
            if byte == '"' {
                new_value.push('\'').unwrap();
            } else {
                new_value.push(byte).unwrap();
            }
        }

        self.value = new_value;
    }

    fn wrap_and_sanitize_value(&mut self) {
        let mut new_value: String<U256> = String::new();
        new_value.push('\'').unwrap();
        for byte in self.value.as_str().chars() {
            if byte == '"' {
                new_value.push('\'').unwrap();
            } else {
                new_value.push(byte).unwrap();
            }
        }
        new_value.push('\'').unwrap();

        self.value = new_value;
    }

    pub fn success<'a, 'b>(attribute: &'a str, value: &'b str) -> Self
    {
        let mut res = Self { code: 200, attribute: String::from(attribute), value: String::from(value)};
        res.sanitize_value();
        res
    }

    pub fn error<'a, 'b>(attribute: &'a str, message: &'b str) -> Self
    {
        let mut res = Self { code: 400, attribute: String::from(attribute), value: String::from(message)};
        res.wrap_and_sanitize_value();
        res
    }

    pub fn custom<'a>(code: i32, message : &'a str) -> Self
    {
        let mut res = Self { code: code, attribute: String::from(""), value: String::from(message)};
        res.wrap_and_sanitize_value();
        res
    }
}

#[derive(Serialize)]
pub struct Status {
    pub t: u32,
    pub x0: f32,
    pub y0: f32,
    pub x1: f32,
    pub y1: f32,
}

pub fn json_reply<T: Serialize>(socket: &mut net::socket::TcpSocket, msg: &T) {
    let mut u: String<U512> = to_string(msg).unwrap();
    u.push('\n').unwrap();
    socket.write_str(&u).unwrap();
}

pub struct Server {
    data: Vec<u8, U256>,
    discard: bool,
}

impl Server {
    pub fn new() -> Self {
        Self {
            data: Vec::new(),
            discard: false,
        }
    }

    pub fn poll<F>(
        &mut self,
        socket: &mut net::socket::TcpSocket,
        mut f: F,
    )
    where
        F: FnMut(&Request) -> Response
    {
        while socket.can_recv() {
            let found = socket.recv(|buf| {
                let (len, found) =
                    match buf.iter().position(|&c| c as char == '\n') {
                        Some(end) => (end + 1, true),
                        None => (buf.len(), false),
                    };
                if self.data.len() + len >= self.data.capacity() {
                    self.discard = true;
                    self.data.clear();
                } else if !self.discard && len > 0 {
                    self.data.extend_from_slice(&buf[..len]).unwrap();
                }
                (len, found)
            }).unwrap();

            if found {
                if self.discard {
                    self.discard = false;
                    json_reply(socket, &Response::custom(520, "command buffer overflow"));
                } else {
                    let r = from_slice::<Request>(&self.data[..self.data.len() - 1]);
                    match r {
                        Ok(mut res) => {
                            res.restore_value();
                            let response = f(&res);
                            json_reply(socket, &response);
                        },
                        Err(err) => {
                            warn!("parse error {:?}", err);
                            json_reply(socket, &Response::custom(550, "parse error"));
                        }
                    }
                }
                self.data.clear();
            }
        }
    }
}

